#!/usr/bin/env python3
"""DDS 互通实验（MVP）：一条命令完成 Native ⇄ ROS 2 的双向验证并产出报告。

做四件事（每条都断言字段值，不只是"能连上"）：

  1. Native → ROS   envelope
  2. ROS → Native   envelope
  3. Native → ROS   probe（嵌套 Header + 自定义消息数组 + string + 原生数组）
  4. ROS → Native   probe

用法：
  python3 scripts/dds_interop_check.py \
      --native build/dds/interop_native \
      --ros-setup /opt/ros/jazzy/setup.bash \
      --ws-setup /ws/install/setup.bash

依赖：本机有 ROS 2 + rmw_cyclonedds_cpp；本脚本用 subprocess 调用 ros2 CLI。
退出码：0 全部通过；1 有失败项；2 环境问题。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ENVELOPE_ROS = "/motionslam/interop/envelope"
ENVELOPE_DDS = "rt/motionslam/interop/envelope"
PROBE_ROS = "/motionslam/interop/probe"
PROBE_DDS = "rt/motionslam/interop/probe"


class Runner:
    def __init__(self, ros_setup: str, ws_setup: str) -> None:
        self.env = dict(os.environ)
        self.env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        self.env.setdefault("ROS_DOMAIN_ID", "0")
        self.ros_setup = ros_setup
        self.ws_setup = ws_setup

    def _prelude(self) -> str:
        parts = [f"source {self.ros_setup}"]
        if self.ws_setup:
            parts.append(f"[ -f {self.ws_setup} ] && source {self.ws_setup}")
        return " && ".join(parts)

    def ros(self, cmd: str, timeout: float) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["bash", "-lc", f"{self._prelude()} && {cmd}"],
            env=self.env, capture_output=True, text=True, timeout=timeout,
        )

    def spawn_ros(self, cmd: str, out_path: Path) -> subprocess.Popen:
        fh = out_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            ["bash", "-lc", f"{self._prelude()} && {cmd}"],
            env=self.env, stdout=fh, stderr=subprocess.STDOUT, text=True,
        )
        proc._fh = fh  # type: ignore[attr-defined]
        return proc


def run_native(native: Path, args: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run([str(native), *args], capture_output=True, text=True, timeout=timeout)


def wait_for(predicate, timeout_s: float, interval_s: float = 0.2) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return False


def native_pub_to_ros(runner: Runner, native: Path, typ: str, ros_topic: str, dds_topic: str,
                      ros_type: str, expect: list[str], workdir: Path) -> dict:
    echo_log = workdir / f"echo_{typ}.log"
    proc = runner.spawn_ros(f"ros2 topic echo --once {ros_topic} {ros_type}", echo_log)
    time.sleep(2.5)  # 让 ROS 订阅先建立
    try:
        res = run_native(native, ["--type", typ, "--role", "pub", "--topic", dds_topic,
                                  "--timeout-ms", "10000"], timeout=30)
        proc.wait(timeout=20)
    finally:
        if proc.poll() is None:
            proc.kill()
        proc._fh.close()  # type: ignore[attr-defined]

    text = echo_log.read_text(encoding="utf-8", errors="replace")
    missing = [item for item in expect if item not in text]
    return {
        "direction": "native_to_ros",
        "type": typ,
        "native_rc": res.returncode,
        "native_stdout": res.stdout.strip(),
        "ros_output_len": len(text),
        "expect_all_present": not missing,
        "missing": missing,
        "ok": res.returncode == 0 and not missing,
    }


def ros_pub_to_native(runner: Runner, native: Path, typ: str, ros_topic: str, dds_topic: str,
                      ros_type: str, payload: str, expect: list[str], workdir: Path) -> dict:
    sub_log = workdir / f"native_sub_{typ}.log"
    fh = sub_log.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        [str(native), "--type", typ, "--role", "sub", "--topic", dds_topic,
         "--count", "1", "--timeout-ms", "20000"],
        stdout=fh, stderr=subprocess.STDOUT, text=True,
    )
    time.sleep(2.0)  # 让原生 reader 先建立
    pub = runner.ros(f'ros2 topic pub --once {ros_topic} {ros_type} \'{payload}\'', timeout=40)
    try:
        proc.wait(timeout=25)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    fh.close()

    text = sub_log.read_text(encoding="utf-8", errors="replace")
    missing = [item for item in expect if item not in text]
    return {
        "direction": "ros_to_native",
        "type": typ,
        "native_rc": proc.returncode,
        "ros_pub_rc": pub.returncode,
        "ros_pub_stderr": pub.stderr.strip()[:400],
        "native_output": text.strip(),
        "expect_all_present": not missing,
        "missing": missing,
        "ok": proc.returncode == 0 and pub.returncode == 0 and not missing,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--native", default="build/dds/interop_native")
    ap.add_argument("--ros-setup", default="/opt/ros/jazzy/setup.bash")
    ap.add_argument("--ws-setup", default="")
    ap.add_argument("--report-dir", default="dds/reports")
    args = ap.parse_args(argv)

    native = Path(args.native)
    if not native.is_absolute():
        native = Path.cwd() / native
    if not native.is_file():
        print(f"缺少 native 可执行文件：{native}（先 cmake --build ... --target interop_native）",
              file=sys.stderr)
        return 2
    if not Path(args.ros_setup).is_file():
        print(f"缺少 ROS setup：{args.ros_setup}", file=sys.stderr)
        return 2

    runner = Runner(args.ros_setup, args.ws_setup)
    env_info = {
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "rmw": "rmw_cyclonedds_cpp",
        "domain_id": runner.env.get("ROS_DOMAIN_ID", "0"),
        "cyclonedds": "",
        "native_bin": str(native),
    }
    try:
        v = runner.ros("ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null && "
                       "dpkg -l ros-jazzy-cyclonedds 2>/dev/null | tail -1", timeout=60)
        env_info["cyclonedds"] = v.stdout.strip().splitlines()[-1] if v.stdout.strip() else ""
        env_info["ros_distro"] = (runner.ros("echo $ROS_DISTRO", timeout=30).stdout or "").strip()
    except subprocess.TimeoutExpired:
        pass

    results: list[dict] = []
    with tempfile.TemporaryDirectory() as tmp:
        workdir = Path(tmp)

        results.append(native_pub_to_ros(
            runner, native, "envelope", ENVELOPE_ROS, ENVELOPE_DDS,
            "motionslam_msgs/msg/RuntimeEnvelope",
            ["schema_version: 1", "channel_id: 7"],
            workdir))
        results.append(ros_pub_to_native(
            runner, native, "envelope", ENVELOPE_ROS, ENVELOPE_DDS,
            "motionslam_msgs/msg/RuntimeEnvelope",
            "{schema_version: 5, channel_id: 99, payload: [72, 105], stamp_ns: 42}",
            ['"schema_version": 5', '"channel_id": 99', '"payload": "Hi"'],
            workdir))
        results.append(native_pub_to_ros(
            runner, native, "probe", PROBE_ROS, PROBE_DDS,
            "motionslam_msgs/msg/TrajectoryProbe",
            ["proposal_id: probe-42", "frame_id: map", "y: -1.25", "kind: 7"],
            workdir))
        results.append(ros_pub_to_native(
            runner, native, "probe", PROBE_ROS, PROBE_DDS,
            "motionslam_msgs/msg/TrajectoryProbe",
            ("{header: {stamp: {sec: 9, nanosec: 11}, frame_id: odom}, "
             "proposal_id: from-ros, model_id: planner-x, "
             "stamps_s: [1.5, 2.5], "
             "points: [{x: 0.5, y: 0.25, z: 0.0, yaw_deg: 45.0}], kind: 3, ok: false}"),
            ['"frame_id": "odom"', '"proposal_id": "from-ros"', '"kind": 3', '"ok": false'],
            workdir))

    passed = sum(1 for r in results if r["ok"])
    total = len(results)
    report = {
        "generated_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "environment": env_info,
        "summary": {"passed": passed, "total": total, "ok": passed == total},
        "results": results,
    }

    report_dir = Path(args.report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "dds_interop_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# DDS 互通实验报告",
        "",
        f"- 时间：{report['generated_at']}",
        f"- 环境：ROS {env_info['ros_distro']} · {env_info['rmw']} · domain {env_info['domain_id']}",
        f"- CycloneDDS：{env_info['cyclonedds'] or 'n/a'}",
        f"- 结论：**{passed}/{total} 通过**",
        "",
        "| 方向 | 类型 | 结果 | 细节 |",
        "| --- | --- | --- | --- |",
    ]
    for r in results:
        detail = "全部字段匹配" if r["ok"] else f"缺失：{r['missing']}"
        lines.append(f"| {r['direction']} | {r['type']} | "
                     f"{'PASS' if r['ok'] else 'FAIL'} | {detail} |")
    (report_dir / "dds_interop_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\n".join(lines))
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
