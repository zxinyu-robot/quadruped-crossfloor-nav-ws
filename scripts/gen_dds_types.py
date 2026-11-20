#!/usr/bin/env python3
"""从 .msg 的 ROSIDL 导出物生成 CycloneDDS 原生类型支持（A1 路线）。

流程（单一类型来源 = `.msg`，本脚本只做确定性代码生成）：

    pkg/msg/Type.msg                                   （作者唯一编辑的源）
        └─ rosidl_adapter ──► dds/rosidl/pkg/msg/Type.idl   （ROS 导出快照，入库）
              └─ transform_rosidl_idl.py ──► dds/idl/pkg/msg/Type.idl
                    └─ idlc ──────────────► dds/gen/<pkg>/Type.{c,h}
                          └─ 断言 m_typename == "pkg::msg::dds_::Type_"

入库的三层产物都是**生成物**（manifest.json 记录 SHA-256 与 idlc 版本），
`.msg` 仍是唯一人工维护的定义；`--check` 可在没有 ROS 的机器上跑。

用法：
  python3 scripts/gen_dds_types.py --msg motionslam_msgs/msg/RuntimeEnvelope
  python3 scripts/gen_dds_types.py --refresh-rosidl   # 从 ROS share 刷新导出快照
  python3 scripts/gen_dds_types.py --check            # CI：校验生成物与快照同步
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

from transform_rosidl_idl import transform  # noqa: E402

INCLUDE_RE = re.compile(r'^#include\s+"([^"]+)"', re.M)
TYPENAME_RE = re.compile(r'\.m_typename\s*=\s*"([^"]+)"')
GUARD_RE = re.compile(r"DDSC__[A-Z0-9_]+")
FLAGSET_RE = re.compile(r"\.m_flagset\s*=\s*([A-Za-z0-9_ |]+)")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def find_idl(msg: str, roots: list[Path]) -> Path:
    """在给定 root 下定位 `pkg/msg/Type.idl`。"""
    pkg, _, typ = msg.partition("/msg/")
    candidates = []
    for root in roots:
        candidates.append(root / pkg / "msg" / f"{typ}.idl")
        candidates.append(root / pkg / "rosidl_adapter" / pkg / "msg" / f"{typ}.idl")
    for cand in candidates:
        if cand.is_file():
            return cand
    raise SystemExit(f"找不到 {msg} 的 .idl；已尝试：\n  " + "\n  ".join(map(str, candidates)))


def resolve_closure(msg: str, roots: list[Path]) -> list[str]:
    """递归解析 #include 依赖，返回按拓扑序（依赖在前）的消息列表。"""
    seen: list[str] = []
    visiting: set[str] = set()

    def visit(current: str) -> None:
        if current in seen:
            return
        if current in visiting:
            raise SystemExit(f"IDL 循环依赖：{current}")
        visiting.add(current)
        src = find_idl(current, roots)
        for inc in INCLUDE_RE.findall(src.read_text(encoding="utf-8")):
            dep = inc[: -len(".idl")]  # pkg/msg/Type.idl -> pkg/msg/Type
            visit(dep)
        visiting.discard(current)
        seen.append(current)

    visit(msg)
    return seen


def idlc_version(idlc: Path) -> str:
    try:
        out = subprocess.run([str(idlc), "-v"], capture_output=True, text=True, check=False)
        return (out.stdout or out.stderr).strip().splitlines()[0]
    except OSError as exc:
        raise SystemExit(f"idlc 不可执行：{idlc}（{exc}）") from exc


def build(dds_root: Path, msgs: list[str], roots: list[Path], idlc: Path,
          typeinfo: bool) -> dict:
    idl_dir = dds_root / "idl"
    gen_dir = dds_root / "gen"
    for stale in (idl_dir, gen_dir):
        if stale.exists():
            shutil.rmtree(stale)
    idl_dir.mkdir(parents=True)
    gen_dir.mkdir(parents=True)

    version = idlc_version(idlc)
    manifest: dict = {"generator": "gen_dds_types.py", "idlc": version, "messages": {}}

    ordered: list[str] = []
    for msg in msgs:
        for item in resolve_closure(msg, roots):
            if item not in ordered:
                ordered.append(item)

    for msg in ordered:
        pkg, _, typ = msg.partition("/msg/")
        src = find_idl(msg, roots)
        dst = idl_dir / pkg / "msg" / f"{typ}.idl"
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(transform(src.read_text(encoding="utf-8"), pkg, typ), encoding="utf-8")

        pkg_gen = gen_dir / pkg
        pkg_gen.mkdir(parents=True, exist_ok=True)
        cmd = [str(idlc), "-I", str(idl_dir), "-o", str(pkg_gen), "-l", "c"]
        if not typeinfo:
            cmd.append("-t")
        cmd.append(str(dst))
        res = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if res.returncode != 0:
            raise SystemExit(f"idlc 失败 {msg}:\n{res.stdout}\n{res.stderr}")

        c_file = pkg_gen / f"{typ}.c"
        h_file = pkg_gen / f"{typ}.h"
        if not c_file.is_file():
            raise SystemExit(f"idlc 未生成 {c_file}")

        # idlc 把绝对输出/输入路径写进文件头注释，导致换目录生成时字节不同。
        # 归一化成 <DDS_ROOT>，保证生成物可复现、--check 可跨机器比对。
        # 另外 idlc 用「绝对输出路径」生成 include guard，换目录就变；改成按类型名固定。
        guard = f"DDSC__{pkg.upper()}__{typ.upper()}_H"
        for path in (c_file, h_file):
            body = path.read_text(encoding="utf-8", errors="replace")
            body = body.replace(str(dds_root), "<DDS_ROOT>")
            body = GUARD_RE.sub(guard, body)
            path.write_text(body, encoding="utf-8")

        c_text = c_file.read_text(encoding="utf-8", errors="replace")
        found = TYPENAME_RE.findall(c_text)
        expected = f"{pkg}::msg::dds_::{typ}_"
        if expected not in found:
            raise SystemExit(
                f"类型名断言失败 {msg}：期望 {expected}，实际 {found}"
            )
        flags = FLAGSET_RE.findall(c_text)

        manifest["messages"][msg] = {
            "source_idl": f"{pkg}/msg/{typ}.idl",
            "source_sha256": sha256(src),
            "transformed_sha256": sha256(dst),
            "generated": {
                "c": sha256(c_file),
                "h": sha256(h_file),
            },
            "m_typename": expected,
            "m_flagset": flags[0].strip() if flags else "",
        }
    manifest["order"] = ordered
    (dds_root / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def check_without_idlc(dds_root: Path, msgs: list[str], roots: list[Path]) -> int:
    """无 idlc 的 CI 环境：只校验转换结果与 manifest 哈希一致。"""
    manifest_path = dds_root / "manifest.json"
    if not manifest_path.is_file():
        print(f"缺少 {manifest_path}", file=sys.stderr)
        return 1
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    drift: list[str] = []
    for msg in msgs:
        pkg, _, typ = msg.partition("/msg/")
        src = find_idl(msg, roots)
        expected = transform(src.read_text(encoding="utf-8"), pkg, typ)
        target = dds_root / "idl" / pkg / "msg" / f"{typ}.idl"
        if not target.is_file() or target.read_text(encoding="utf-8") != expected:
            drift.append(f"idl/{pkg}/msg/{typ}.idl")
        entry = manifest.get("messages", {}).get(msg, {})
        if entry.get("source_sha256") != sha256(src):
            drift.append(f"manifest:source_sha256({msg})")
        if entry.get("m_typename") != f"{pkg}::msg::dds_::{typ}_":
            drift.append(f"manifest:m_typename({msg})")
    if drift:
        print("生成物与源不同步（无 idlc 模式）：", file=sys.stderr)
        for item in drift:
            print(f"  {item}", file=sys.stderr)
        return 1
    print(f"dds types in sync ({len(msgs)} msg, transform-only mode)")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--msg", action="append", default=[], help="pkg/msg/Type，可重复")
    ap.add_argument("--dds-root", default=str(ROOT / "dds"))
    ap.add_argument("--share-root", action="append", default=[],
                    help="搜索 .idl 的根目录，可重复（默认 ROS share + 构建目录）")
    ap.add_argument("--idlc", default="idlc")
    ap.add_argument("--check", action="store_true", help="仅校验已提交生成物是否同步")
    ap.add_argument("--refresh-rosidl", action="store_true",
                    help="从 --share-root 刷新 dds/rosidl 导出快照")
    ap.add_argument("--no-typeinfo", action="store_true", help="idlc -t：不生成 XTypes 元数据")
    args = ap.parse_args(argv)

    dds_root = Path(args.dds_root)
    if not dds_root.is_absolute():
        dds_root = ROOT / dds_root
    snapshot = dds_root / "rosidl"

    if args.refresh_rosidl:
        share_roots = [Path(p) for p in args.share_root]
        if not share_roots:
            distro_share = sorted(Path("/opt/ros").glob("*/share")) if Path("/opt/ros").is_dir() else []
            share_roots = distro_share + [Path("/ws/build")]
        share_roots = [p for p in share_roots if p.is_dir()]
        if not share_roots:
            raise SystemExit("--refresh-rosidl 需要 --share-root")
        refreshed = 0
        for msg in args.msg or []:
            pkg, _, typ = msg.partition("/msg/")
            src = find_idl(msg, share_roots)
            dst = snapshot / pkg / "msg" / f"{typ}.idl"
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
            refreshed += 1
        print(f"refreshed {refreshed} idl snapshot(s) -> {snapshot}")
        return 0

    roots = [snapshot]
    if not snapshot.is_dir():
        raise SystemExit(f"缺少 ROSIDL 导出快照：{snapshot}（先跑 --refresh-rosidl）")

    msgs = args.msg
    if not msgs:
        msgs = sorted(
            f"{pkgdir.name}/msg/{f.stem}"
            for pkgdir in snapshot.glob("*")
            if pkgdir.is_dir() and (pkgdir / "msg").is_dir()
            for f in sorted((pkgdir / "msg").glob("*.idl"))
            if "__" not in f.name
        )
    if not msgs:
        raise SystemExit("没有待生成的消息（用 --msg 指定，或在 dds/types/<pkg>/*.msg 放置）")

    idlc = Path(args.idlc)
    if not idlc.is_file():
        resolved = shutil.which(str(idlc))
        idlc = Path(resolved) if resolved else None
    if idlc is None and not args.check:
        raise SystemExit(f"找不到 idlc：{args.idlc}（--check 模式可缺省）")

    if args.check:
        # 无 idlc 时降级为「转换层 + manifest 哈希」校验：仍能拦截 .msg/IDL 漂移
        if idlc is None:
            failed = check_without_idlc(dds_root, msgs, roots)
            return failed
        with tempfile.TemporaryDirectory() as tmp:
            fresh = Path(tmp) / "dds"
            build(fresh, msgs, roots, idlc, not args.no_typeinfo)
            drift = []
            for path in sorted(fresh.rglob("*")):
                if not path.is_file():
                    continue
                rel = path.relative_to(fresh)
                target = dds_root / rel
                if not target.is_file() or target.read_bytes() != path.read_bytes():
                    drift.append(str(rel))
            if drift:
                print("生成物与源不同步：", file=sys.stderr)
                for item in drift:
                    print(f"  {item}", file=sys.stderr)
                print("运行 python3 scripts/gen_dds_types.py --msg … 后提交", file=sys.stderr)
                return 1
            print(f"dds types in sync ({len(msgs)} msg)")
            return 0

    manifest = build(dds_root, msgs, roots, idlc, not args.no_typeinfo)
    print(f"generated {len(manifest['messages'])} message type(s) -> {dds_root}")
    for msg, info in manifest["messages"].items():
        print(f"  {msg}: m_typename={info['m_typename']} flagset={info['m_flagset'] or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
