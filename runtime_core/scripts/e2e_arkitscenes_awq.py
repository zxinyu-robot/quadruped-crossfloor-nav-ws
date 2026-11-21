#!/usr/bin/env python3
"""Fastest closed loop: ARKitScenes frames -> Qwen2.5-VL-7B-AWQ (vLLM)
-> SemanticObservation -> VisionLoop -> TaskRuntime -> feedback.
author: zxinyu <zxiangsheng2314@gmail.com>
"""
import glob, json, os, subprocess, sys, time

sys.path.insert(0, "/root/ds/ln_src")


def main():
    from vllm import LLM, SamplingParams
    from PIL import Image
    import numpy as np
    from legged_nav.spatial_context import StreamContext
    from legged_nav.vision_loop import VisionContext, VisionLoop
    from legged_nav.vlm_adapter import SemanticObservation
    from legged_nav.runtime import TaskRuntime
    from legged_nav.executor import SimForwarder
    from legged_nav.spatial_state import (blocking_hazards,
                                          yaw_from_axis_angle)

    root = "/root/ms/models/Qwen--Qwen2.5-VL-7B-Instruct-AWQ"
    snap = subprocess.check_output(
        ["bash", "-c", f"find {root} -name config.json | head -1 | xargs dirname"]
    ).decode().strip()
    DIR = "/root/ds/arkitscenes"

    PROMPT = (
        "You are a quadruped robot's vision-language perception module. "
        "Reply with ONLY a JSON object: "
        '{"scene_class": corridor|office|kitchen|bedroom|livingroom|elevator|stairs|unknown, '
        '"confidence": 0-1, "landmarks": [str], "hazards": [str], '
        '"target_visible": true|false, "summary": short}. '
        "Mission: navigate to an office and return."
    )

    llm = LLM(model=snap, quantization="awq_marlin", max_model_len=4096,
              limit_mm_per_prompt={"image": 1, "video": 0},
              gpu_memory_utilization=0.85)
    sp = SamplingParams(max_tokens=160, temperature=0)

    class VLLMBackend:
        def observe(self, frame, task_prompt, frame_id=""):
            prompt = ("<|im_start|>user\n<|vision_start|><|image_pad|><|vision_end|>"
                      + PROMPT + "<|im_end|>\n<|im_start|>assistant\n")
            t1 = time.time()
            text = llm.generate(
                [{"prompt": prompt,
                  "multi_modal_data": {"image": [Image.open(frame)]}}],
                sp)[0].outputs[0].text
            try:
                d = json.loads(text[text.index("{"):text.rindex("}") + 1])
            except Exception:
                d = {}
            # 深度门控：最近邻 lowres_depth 帧（uint16 毫米），
            # 前向走廊近场最小深度 < 0.9m 才认定 hazards 阻挡通行
            ts = float(os.path.basename(frame).split("_")[1][:-4])
            depth = None
            if dts:
                dp = dts[min(dts, key=lambda t: abs(t - ts))]
                depth = np.array(Image.open(dp))
            obs_hazards = blocking_hazards(
                depth,
                [str(x) for x in d.get("hazards", [])
                 if str(x).lower() not in ("no visible hazards", "none")])
            return SemanticObservation(
                scene_class=str(d.get("scene_class", "unknown")),
                confidence=float(d.get("confidence", 0.0)),
                landmarks=[str(x) for x in d.get("landmarks", [])],
                hazards=obs_hazards,
                target_visible=bool(d.get("target_visible", False)),
                model_id="Qwen2.5-VL-7B-Instruct-AWQ",
                frame_id=frame_id, raw_text=text[:300])

    backend = VLLMBackend()
    backend.lat = []

    # 深度帧与 RGB 不同步，按时间戳最近邻匹配
    dts = {}
    for dp in glob.glob(os.path.join(DIR, "lowres_depth", "*.png")):
        dts[float(os.path.basename(dp).split("_")[1][:-4])] = dp

    # traj 实测列序：ts tx ty tz rx ry rz（轴角为弧度，平移为米）；
    # yaw 由轴角经 Rodrigues 投影得到。
    poses = {}
    for ln in open(os.path.join(DIR, "lowres_wide.traj")):
        p = ln.split()
        if len(p) >= 7:
            yaw = yaw_from_axis_angle(float(p[4]), float(p[5]), float(p[6]))
            poses[float(p[0])] = (float(p[1]), float(p[2]), float(p[3]), yaw)

    fwd = SimForwarder()
    rt = TaskRuntime(fwd, local_platform="unitree_go2_edu",
                     local_robot_id="go2_001")
    ctx = VisionContext(
        task_prompt="navigate to an office and return",
        robot={"platform": "unitree_go2_edu", "sn": "go2_001"},
        world_ref={"map_id": "map_01", "floor_id": "floor_01",
                   "frame_id": "world", "revision": 3},
        goal={"x": 1.5, "y": -1.0, "z": 0.0})
    stream = StreamContext(window=4, dedup_dist_m=0.3)
    loop = VisionLoop(backend, rt, ctx, stream=stream)

    seq = 0
    for fp in sorted(glob.glob(os.path.join(DIR, "*.png"))):
        ts = float(os.path.basename(fp).split("_")[1][:-4])
        pt = min(poses, key=lambda t: abs(t - ts))
        seq += 1
        t1 = time.time()
        dec = loop.step(frame=fp, frame_id=os.path.basename(fp),
                        seq=seq, pose=poses[pt][:3])
        lat = round(time.time() - t1, 2)
        backend.lat.append(lat)
        fb = dec.feedback or {}
        rec = {"seq": seq, "frame": os.path.basename(fp),
               "pose": poses[pt][:3], "yaw_deg": poses[pt][3],
               "scene": dec.observation.scene_class,
               "conf": dec.observation.confidence,
               "hazards": dec.observation.hazards,
               "target": dec.observation.target_visible,
               "intent": (dec.intent["skills"][0]["name"]
                          if dec.intent else None),
               "fb_state": fb.get("state"), "fb_event": fb.get("event"),
               "reason": dec.reason, "latency_s": lat}
        print(json.dumps(rec, ensure_ascii=False), flush=True)

    st = stream.stats()
    print(json.dumps({"n": seq,
                      "vlm_p50_s": sorted(backend.lat)[len(backend.lat)//2],
                      "fast": st["fast_len"], "slow": st["slow_compressed"],
                      "dedup_dropped": st["dedup_dropped"],
                      "kv_reuse": st["kv_reuse_ratio"],
                      "odom_m": st["slow_odom_m"],
                      "runtime_events": len(rt.events)}))


if __name__ == "__main__":
    main()
