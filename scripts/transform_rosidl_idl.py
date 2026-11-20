#!/usr/bin/env python3
"""把 ROSIDL 导出的 .idl 确定性地转换成 CycloneDDS 可用的 .idl。

为什么需要这一步
----------------
rosidl 为 `pkg/msg/Type.msg` 导出的 IDL 是：

    module pkg { module msg { struct Type { ... }; }; };

而 rmw_cyclonedds_cpp 在 DDS 上公告的类型名是：

    pkg::msg::dds_::Type_

两者不同，直接交给 idlc 会生成错误的 `m_typename`，导致与 ROS 端点无法匹配
（实测：topic 名和类型名任一不符即 `matched_subscribers=0`）。

转换规则（全部基于 IDL 语法结构，不做字符串替换）
--------------------------------------------------
1. 删除 `@verbatim(...)` 等注解（含多行字符串拼接），避免注释污染生成物。
2. 每个 `struct Type { ... };` 包进 `module dds_ { struct Type_ { ... }; };`。
3. 结构体内的类型引用 `pkg::msg::Type` → `pkg::msg::dds_::Type_`。
4. `#include` 原样保留（调用方用 `-I` 指向同布局的输出目录即可解析）。

只做这三类结构改写，其余 token 原样保留（含 typedef、constants 模块）。
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass

# ---------------------------------------------------------------- tokenizer

IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
NUM_RE = re.compile(r"[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?")

NO_SPACE_BEFORE = {";", ",", "]", ">", "[", "(", ")"}
NO_SPACE_AFTER = {"[", "<", "(", "::", "@"}
NO_SPACE_BOTH = {"::"}


@dataclass
class Tok:
    kind: str  # ppd | anno | ident | num | str | punct
    text: str


def tokenize(text: str) -> list[Tok]:
    toks: list[Tok] = []
    i, n = 0, len(text)
    at_line_start = True
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            if c == "\n":
                at_line_start = True
            i += 1
            continue
        if at_line_start and c == "#":
            j = text.find("\n", i)
            j = n if j < 0 else j
            toks.append(Tok("ppd", text[i:j].strip()))
            i = j
            at_line_start = True
            continue
        at_line_start = False
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
            continue
        if c == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    j += 1
                    break
                j += 1
            toks.append(Tok("str", text[i:j]))
            i = j
            continue
        if c == "@":
            m = IDENT_RE.match(text, i + 1)
            if m:
                toks.append(Tok("anno", "@" + m.group(0)))
                i = m.end()
                continue
            toks.append(Tok("punct", c))
            i += 1
            continue
        if text.startswith("::", i):
            toks.append(Tok("punct", "::"))
            i += 2
            continue
        m = IDENT_RE.match(text, i)
        if m:
            toks.append(Tok("ident", m.group(0)))
            i = m.end()
            continue
        m = NUM_RE.match(text, i)
        if m:
            toks.append(Tok("num", m.group(0)))
            i = m.end()
            continue
        toks.append(Tok("punct", c))
        i += 1
    return toks


def join(toks: list[Tok]) -> str:
    out: list[str] = []
    prev: Tok | None = None
    for t in toks:
        if prev is not None:
            need_space = True
            if prev.text in NO_SPACE_AFTER or t.text in NO_SPACE_BEFORE:
                need_space = False
            if prev.text in NO_SPACE_BOTH or t.text in NO_SPACE_BOTH:
                need_space = False
            if t.kind == "ppd":
                need_space = False
                out.append("\n")
            if need_space:
                out.append(" ")
        out.append(t.text)
        prev = t
    return "".join(out)


def match_paren(toks: list[Tok], start: int) -> int:
    """`toks[start]` 是 '('，返回匹配的 ')' 下标。"""
    depth = 0
    for k in range(start, len(toks)):
        if toks[k].text == "(":
            depth += 1
        elif toks[k].text == ")":
            depth -= 1
            if depth == 0:
                return k
    raise ValueError("unbalanced parentheses in annotation")


def match_brace(toks: list[Tok], start: int) -> int:
    depth = 0
    for k in range(start, len(toks)):
        if toks[k].text == "{":
            depth += 1
        elif toks[k].text == "}":
            depth -= 1
            if depth == 0:
                return k
    raise ValueError("unbalanced braces in struct")


def strip_annotations(toks: list[Tok]) -> list[Tok]:
    """删除注解；`@x(...)` 连同参数一起删。"""
    out: list[Tok] = []
    i = 0
    while i < len(toks):
        t = toks[i]
        if t.kind == "anno":
            i += 1
            if i < len(toks) and toks[i].text == "(":
                i = match_paren(toks, i) + 1
            continue
        out.append(t)
        i += 1
    return out


def rewrite_refs(toks: list[Tok]) -> list[Tok]:
    """`a::msg::b` → `a::msg::dds_::b_`；已是 dds_ 形式的不动。"""
    out: list[Tok] = []
    i = 0
    while i < len(toks):
        if (
            toks[i].kind == "ident"
            and i + 4 < len(toks) + 0
            and toks[i + 1].text == "::"
            and toks[i + 2].kind == "ident"
            and toks[i + 3].text == "::"
            and i + 4 < len(toks)
            and toks[i + 4].kind == "ident"
        ):
            pkg, mid, name = toks[i].text, toks[i + 2].text, toks[i + 4].text
            if mid == "msg" and name != "dds_" and not name.endswith("_"):
                out.extend(
                    [
                        toks[i],
                        toks[i + 1],
                        toks[i + 2],
                        Tok("punct", "::"),
                        Tok("ident", "dds_"),
                        Tok("punct", "::"),
                        Tok("ident", name + "_"),
                    ]
                )
                i += 5
                continue
            if mid == "msg" and name == "dds_":
                # 已是 a::msg::dds_::b 形式，只需给尾名补下划线
                out.extend(toks[i : i + 5])
                i += 5
                if i < len(toks) and toks[i].text == "::" and i + 1 < len(toks):
                    out.extend([toks[i], toks[i + 1]])
                    i += 2
                continue
        out.append(toks[i])
        i += 1
    return out


def transform(text: str, package: str, type_name: str) -> str:
    toks = strip_annotations(tokenize(text))

    header: list[Tok] = []
    body: list[Tok] = []
    for t in toks:
        (header if t.kind == "ppd" else body).append(t)

    out: list[Tok] = list(header)

    i = 0
    while i < len(body):
        t = body[i]
        if t.kind == "ident" and t.text == "struct" and i + 2 < len(body):
            name_tok = body[i + 1]
            open_idx = i + 2
            if body[open_idx].text != "{":
                out.append(t)
                i += 1
                continue
            close_idx = match_brace(body, open_idx)
            inner = rewrite_refs(body[open_idx + 1 : close_idx])
            out.extend(
                [
                    Tok("ident", "module"),
                    Tok("ident", "dds_"),
                    Tok("punct", "{"),
                    Tok("ident", "struct"),
                    Tok("ident", name_tok.text + "_"),
                    Tok("punct", "{"),
                ]
            )
            out.extend(inner)
            out.extend(
                [
                    Tok("punct", "}"),
                    Tok("punct", ";"),
                    Tok("punct", "}"),
                    Tok("punct", ";"),
                ]
            )
            i = close_idx + 1
            if i < len(body) and body[i].text == ";":
                i += 1
            continue
        out.append(t)
        i += 1

    text_out = join(out)
    expected = f"{package}::msg::dds_::{type_name}_"
    if expected.split("::")[-1] not in text_out:
        raise SystemExit(f"transform error: {type_name} 未出现在输出中")
    return text_out + "\n"


def main(argv: list[str]) -> int:
    if len(argv) != 5:
        print("usage: transform_rosidl_idl.py <in.idl> <out.idl> <package> <TypeName>", file=sys.stderr)
        return 2
    src, dst, package, type_name = argv[1], argv[2], argv[3], argv[4]
    with open(src, encoding="utf-8") as fh:
        text = fh.read()
    result = transform(text, package, type_name)
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(result)
    print(f"transformed {src} -> {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
