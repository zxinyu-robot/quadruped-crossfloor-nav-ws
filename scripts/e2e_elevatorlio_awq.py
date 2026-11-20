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
    from legged_nav.spatial_context import StreamContext
    from legged_nav.vision_loop import VisionContext, VisionLoop
    from legged_nav.vlm_adapter import SemanticObservation
    from legged_nav.runtime import TaskRuntime
    from legged_nav.executor import SimForwarder

    root = "/root/ms/models/Qwen--Qwen2.5-VL-7B-Instruct-AWQ"
    snap = subprocess.check_output(
        ["bash", "-c", f"find {root} -name config.json | head -1 | xargs dirname"]
    ).decode().strip()
    DIR = "/root/gpufree-data/elevatorlio/office3/frames"

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
            return SemanticObservation(
                scene_class=str(d.get("scene_class", "unknown")),
                confidence=float(d.get("confidence", 0.0)),
                landmarks=[str(x) for x in d.get("landmarks", [])],
                hazards=[str(x) for x in d.get("hazards", [])
                         if str(x).lower() not in ("no visible hazards", "none")],
                target_visible=bool(d.get("target_visible", False)),
                model_id="Qwen2.5-VL-7B-Instruct-AWQ",
                frame_id=frame_id, raw_text=text[:300])

    backend = VLLMBackend()
    backend.lat = []

    fwd = SimForwarder()
    poses = {}
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
    for fp in sorted(glob.glob(os.path.join(DIR, "*.jpg"))):
        ts = float(os.path.basename(fp).split("_")[1][:-4])
        pt = None
        seq += 1
        t1 = time.time()
        dec = loop.step(frame=fp, frame_id=os.path.basename(fp),
                        seq=seq, pose=None)
        lat = round(time.time() - t1, 2)
        backend.lat.append(lat)
        fb = dec.feedback or {}
        rec = {"seq": seq, "frame": os.path.basename(fp),
               "pose": None, "scene": dec.observation.scene_class,
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
