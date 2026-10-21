#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "scripts" / "md_to_docx_yahei.py"
DST = ROOT / "scripts" / "md_to_docx_yahei_nomath.py"


def main() -> None:
    text = SRC.read_text(encoding="utf-8")
    text = re.sub(
        r"DELIM_MATH_RE = re\.compile\([\s\S]*?re\.S,\n\)",
        'DELIM_MATH_RE = re.compile(r"(?!x)x")  # disabled: formulas are PNG',
        text,
        count=1,
    )
    text = re.sub(
        r"BARE_MATH_RE = re\.compile\([\s\S]*?\)\n",
        'BARE_MATH_RE = re.compile(r"(?!x)x")  # disabled\n',
        text,
        count=1,
    )
    DST.write_text(text, encoding="utf-8")
    print("wrote", DST)

    md = ROOT / "eye-hand-calibration-guide.md"
    out = ROOT / "eye-hand-calibration-guide.docx"
    out_alt = ROOT / "eye-hand-calibration-guide-v2.docx"

    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("md_to_docx_nomath", DST)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)

    target = out
    try:
        mod.convert(md, target)
        print("saved", target)
    except PermissionError:
        target = out_alt
        mod.convert(md, target)
        print("original locked; saved", target)


if __name__ == "__main__":
    main()
