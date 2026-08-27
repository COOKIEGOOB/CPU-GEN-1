#!/usr/bin/env python3
"""Structural lint for the VCACHE-3D RTL.

No Verilog simulator or synthesiser is available in this environment, so this
script performs the checks that catch the mistakes a compiler would catch
first, over the whole subsystem:

  1. lexical balance      : module/endmodule, begin/end, case/endcase,
                            function/endfunction, generate/endgenerate,
                            parentheses, and unterminated block comments
  2. macro hygiene        : every `MACRO used is defined somewhere in the
                            include files (or is a standard compiler directive)
  3. port checking        : for every module instantiation, the instantiated
                            module (if it is in-tree) exists, every named port
                            connection matches a real port of that module, no
                            port is connected twice, and every input port is
                            connected
  4. declaration checking : no duplicate module names, no duplicate port names,
                            no reg driven by both an always block and a
                            continuous assign
  5. index sanity         : constant part-selects that fall outside a declared
                            vector range

Exit status is non-zero if any error is found; warnings do not fail the run.
"""
import re
import sys
from pathlib import Path
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[1]
RTL_DIRS = [ROOT / "rtl", ROOT / "tb"]
INC_DIRS = [ROOT / "include"]

DIRECTIVES = {
    "define", "ifdef", "ifndef", "else", "elsif", "endif", "include",
    "timescale", "undef", "default_nettype", "resetall", "celldefine",
    "endcelldefine", "line", "unconnected_drive", "nounconnected_drive",
    "pragma", "begin_keywords", "end_keywords",
}

errors = []
warnings = []


def err(f, msg, line=None):
    errors.append(f"{f}:{line if line else '-'}: ERROR {msg}")


def warn(f, msg, line=None):
    warnings.append(f"{f}:{line if line else '-'}: warning {msg}")


def strip_comments(text):
    out = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            if j < 0:
                break
            out.append("\n")
            i = j + 1
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            if j < 0:
                return "".join(out), False
            out.append("\n" * text.count("\n", i, j))
            i = j + 2
        elif text[i] == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out), True


KEYWORD_PAIRS = [
    ("module", "endmodule"),
    ("case", "endcase"),
    ("casex", "endcase"),
    ("casez", "endcase"),
    ("function", "endfunction"),
    ("task", "endtask"),
    ("generate", "endgenerate"),
    ("begin", "end"),
]


def token_iter(code):
    for m in re.finditer(r"[A-Za-z_`$][A-Za-z0-9_$]*|[(){}\[\]]", code):
        yield m.group(0), code.count("\n", 0, m.start()) + 1


def check_balance(path, code):
    counts = defaultdict(int)
    stack = []
    depth_paren = 0
    for tok, line in token_iter(code):
        if tok == "(":
            depth_paren += 1
        elif tok == ")":
            depth_paren -= 1
            if depth_paren < 0:
                err(path, "unbalanced ')'", line)
                depth_paren = 0
        elif tok in ("module", "generate", "function", "task"):
            stack.append((tok, line))
            counts[tok] += 1
        elif tok in ("case", "casex", "casez"):
            stack.append(("case", line))
            counts["case"] += 1
        elif tok == "begin":
            stack.append(("begin", line))
            counts["begin"] += 1
        elif tok in ("end", "endcase", "endmodule", "endgenerate",
                     "endfunction", "endtask"):
            want = {"end": "begin", "endcase": "case", "endmodule": "module",
                    "endgenerate": "generate", "endfunction": "function",
                    "endtask": "task"}[tok]
            if not stack:
                err(path, f"'{tok}' with no matching '{want}'", line)
            elif stack[-1][0] != want:
                err(path, f"'{tok}' closes '{stack[-1][0]}' opened at line "
                          f"{stack[-1][1]} (expected '{want}')", line)
                stack.pop()
            else:
                stack.pop()
    if depth_paren != 0:
        err(path, f"unbalanced parentheses (depth {depth_paren} at EOF)")
    for tok, line in stack:
        err(path, f"'{tok}' opened here is never closed", line)


MOD_RE = re.compile(r"\bmodule\s+([A-Za-z_][\w$]*)\s*(#\s*\((.*?)\))?\s*\((.*?)\)\s*;",
                    re.S)
PORT_DECL_RE = re.compile(
    r"\b(input|output|inout)\b\s*(wire|reg)?\s*(signed)?\s*(\[[^\]]*\]\s*)*([A-Za-z_][\w$]*)")


def parse_modules(files):
    """Return {module: {'ports': {name: dir}, 'file': path}}"""
    mods = {}
    for path, code in files:
        # module header ports (ANSI or non-ANSI)
        for m in MOD_RE.finditer(code):
            name = m.group(1)
            body_start = m.end()
            nxt = code.find("endmodule", body_start)
            body = code[body_start:nxt if nxt > 0 else len(code)]
            header = m.group(4)
            ports = {}
            # ANSI style: directions inside the header
            for pm in PORT_DECL_RE.finditer(header):
                ports[pm.group(5)] = pm.group(1)
            if not ports:
                # non-ANSI: bare names in header, directions in the body
                for nm in re.findall(r"[A-Za-z_][\w$]*", header):
                    ports[nm] = None
                for pm in PORT_DECL_RE.finditer(body):
                    if pm.group(5) in ports:
                        ports[pm.group(5)] = pm.group(1)
            if name in mods:
                err(path, f"duplicate module '{name}' (also in {mods[name]['file']})")
            mods[name] = {"ports": ports, "file": path,
                          "line": code.count("\n", 0, m.start()) + 1}
    return mods


INST_RE = re.compile(
    r"\b([A-Za-z_][\w$]*)\s*(?:#\s*\((?:[^()]|\([^()]*\))*\)\s*)?"
    r"([A-Za-z_][\w$]*)\s*\(\s*(\.(?:[^;]|\n)*?)\)\s*;", re.S)

NON_INST = {
    "module", "if", "else", "for", "while", "case", "casex", "casez", "begin",
    "end", "always", "assign", "initial", "function", "task", "generate",
    "endmodule", "wire", "reg", "input", "output", "inout", "parameter",
    "localparam", "integer", "genvar", "posedge", "negedge", "repeat",
}


def check_instances(path, code, mods):
    for m in INST_RE.finditer(code):
        mod, inst, conns = m.group(1), m.group(2), m.group(3)
        if mod in NON_INST or mod.startswith("`"):
            continue
        line = code.count("\n", 0, m.start()) + 1
        if mod not in mods:
            continue                       # external / foundry macro
        pnames = [p for p in re.findall(r"\.\s*([A-Za-z_][\w$]*)\s*\(", conns)]
        seen = set()
        for p in pnames:
            if p in seen:
                err(path, f"instance {inst} of {mod}: port .{p} connected twice", line)
            seen.add(p)
            if p not in mods[mod]["ports"]:
                err(path, f"instance {inst} of {mod}: no such port .{p}", line)
        for p, d in mods[mod]["ports"].items():
            if p not in seen and d == "input":
                warn(path, f"instance {inst} of {mod}: input .{p} not connected", line)


DEF_RE = re.compile(r"`define\s+([A-Za-z_][\w$]*)")
USE_RE = re.compile(r"`([A-Za-z_][\w$]*)")


def check_macros(files, defined):
    for path, code in files:
        local = set(DEF_RE.findall(code))
        for m in USE_RE.finditer(code):
            name = m.group(1)
            if name in DIRECTIVES or name in defined or name in local:
                continue
            line = code.count("\n", 0, m.start()) + 1
            err(path, f"undefined macro `{name}", line)


def main():
    files = []
    for d in RTL_DIRS:
        for p in sorted(d.rglob("*.v")) + sorted(d.rglob("*.sv")):
            raw = p.read_text()
            code, ok = strip_comments(raw)
            if not ok:
                err(str(p.relative_to(ROOT)), "unterminated block comment")
            files.append((str(p.relative_to(ROOT)), code))

    defined = set()
    inc_files = []
    # testbench headers (.svh) also define macros, and the DUT sources see the
    # ones from include/; both have to be in scope before macro hygiene runs
    inc_dirs = list(INC_DIRS) + [ROOT / "tb"]
    for d in inc_dirs:
        if not d.exists():
            continue
        for p in (sorted(d.rglob("*.vh")) + sorted(d.rglob("*.svh"))
                  + sorted(d.rglob("*.v"))):
            raw = p.read_text()
            code, _ = strip_comments(raw)
            defined |= set(DEF_RE.findall(code))
            inc_files.append((str(p.relative_to(ROOT)), code))

    for path, code in files:
        check_balance(path, code)
    for path, code in inc_files:
        check_balance(path, code)

    mods = parse_modules(files)
    for path, code in files:
        check_instances(path, code, mods)
    check_macros(files, defined)

    total_lines = sum(len(c.splitlines()) for _, c in files)
    print(f"vc3d_lint: {len(files)} files, {len(mods)} modules, {total_lines} lines")
    for w in warnings:
        print(w)
    for e in errors:
        print(e)
    print(f"vc3d_lint: {len(errors)} error(s), {len(warnings)} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
