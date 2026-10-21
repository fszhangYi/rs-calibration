#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render LaTeX formulas to PNG (Codecogs + Pillow fallback)."""
from __future__ import annotations

import urllib.parse
import urllib.request
from io import BytesIO
from pathlib import Path

from PIL import Image, ImageOps

OUT = Path(__file__).resolve().parents[1] / "assets" / "formulas"
OUT.mkdir(parents=True, exist_ok=True)

# (filename_stem, latex without surrounding $)
FORMULAS: list[tuple[str, str]] = [
    ("eq_homogeneous_T", r"T=\begin{bmatrix} R & t \\ 0^{\top} & 1 \end{bmatrix},\quad R\in SO(3),\ t\in\mathbb{R}^{3}"),
    ("eq_point_transform", r"{}^{a}p={}^{a}T_{b}\,{}^{b}p"),
    ("eq_chain_rule", r"{}^{a}T_{c}={}^{a}T_{b}\,{}^{b}T_{c}"),
    ("eq_inverse", r"{}^{b}T_{a}=\left({}^{a}T_{b}\right)^{-1}=\begin{bmatrix} R^{\top} & -R^{\top}t \\ 0^{\top} & 1 \end{bmatrix}"),
    ("eq_axxb", r"A\,X=X\,B"),
    ("eq_eih_A", r"A_{ij}=\left({}^{base}T_{ee}(i)\right)^{-1}{}^{base}T_{ee}(j)"),
    ("eq_eih_B", r"B_{ij}={}^{cam}T_{tgt}(i)\left({}^{cam}T_{tgt}(j)\right)^{-1}"),
    ("eq_eih_AX", r"A_{ij}\,X=X\,B_{ij},\quad X={}^{ee}T_{cam}"),
    ("eq_eih_deploy", r"{}^{base}p={}^{base}T_{ee}\cdot X\cdot{}^{cam}p={}^{base}T_{ee}\cdot{}^{ee}T_{cam}\cdot{}^{cam}p"),
    ("eq_eth_A", r"A_{ij}={}^{base}T_{ee}(j)\left({}^{base}T_{ee}(i)\right)^{-1}"),
    ("eq_eth_B", r"B_{ij}={}^{cam}T_{tgt}(j)\left({}^{cam}T_{tgt}(i)\right)^{-1}"),
    ("eq_eth_AX", r"A_{ij}\,X=X\,B_{ij},\quad X={}^{base}T_{cam}"),
    ("eq_eth_compose", r"{}^{base}T_{cam}={}^{base}T_{ee}\cdot{}^{ee}T_{tgt}\cdot\left({}^{cam}T_{tgt}\right)^{-1}"),
    ("eq_rot", r"R_{A}R_{X}=R_{X}R_{B}"),
    ("eq_trans", r"R_{A}t_{X}+t_{A}=R_{X}t_{B}+t_{X}"),
    ("inline_X_eeTcam", r"X={}^{ee}T_{cam}"),
    ("inline_baseTcam", r"{}^{base}T_{cam}"),
    ("inline_eeTcam", r"{}^{ee}T_{cam}"),
    ("inline_aTb", r"{}^{a}T_{b}"),
    ("inline_baseTee", r"{}^{base}T_{ee}"),
    ("inline_camTtgt", r"{}^{cam}T_{tgt}"),
    ("inline_eeTtgt", r"{}^{ee}T_{tgt}"),
    ("inline_eeTgripper", r"{}^{ee}T_{gripper}"),
    ("inline_baseTtgt", r"{}^{base}T_{tgt}"),
    ("inline_axxb", r"AX=XB"),
    ("inline_AkBk", r"\{A_{k},B_{k}\}_{k=1}^{N}"),
]


def fetch_codecogs(latex: str) -> bytes:
    # Use classic png.latex endpoint; avoid \bg_white (renders as literal text).
    expr = r"\dpi{220} " + latex
    url = "https://latex.codecogs.com/png.latex?" + urllib.parse.quote(expr, safe="")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read()


def fetch_matplotlib(latex: str) -> bytes:
    """Fallback for simple formulas without environments."""
    import matplotlib.pyplot as plt

    # rewrite unsupported constructs lightly
    s = latex
    s = s.replace(r"\begin{bmatrix}", r"\left[")
    s = s.replace(r"\end{bmatrix}", r"\right]")
    s = s.replace(r"\\", r" ; ")
    s = s.replace("&", r"\,")
    fig = plt.figure(figsize=(0.01, 0.01))
    fig.patch.set_facecolor("white")
    try:
        fig.text(0.5, 0.5, f"${s}$", fontsize=20, ha="center", va="center")
        buf = BytesIO()
        fig.savefig(buf, dpi=200, bbox_inches="tight", pad_inches=0.12, facecolor="white")
        plt.close(fig)
        return buf.getvalue()
    except Exception:
        plt.close(fig)
        # last resort: plain text image
        from PIL import ImageDraw, ImageFont

        img = Image.new("RGB", (800, 80), "white")
        draw = ImageDraw.Draw(img)
        draw.text((10, 25), latex[:120], fill=(0, 0, 0))
        buf = BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()


def to_rgb_png(raw: bytes) -> bytes:
    img = Image.open(BytesIO(raw)).convert("RGBA")
    # codecogs often returns transparent bg; flatten to white
    bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
    bg.paste(img, mask=img.split()[-1] if img.mode == "RGBA" else None)
    rgb = bg.convert("RGB")
    # light padding
    rgb = ImageOps.expand(rgb, border=8, fill="white")
    out = BytesIO()
    rgb.save(out, format="PNG", optimize=True)
    return out.getvalue()


def render(stem: str, latex: str) -> None:
    try:
        raw = fetch_codecogs(latex)
        if len(raw) < 100:
            raise RuntimeError("tiny response")
        png = to_rgb_png(raw)
    except Exception as e:
        print(f"  codecogs failed for {stem}: {e}; using matplotlib fallback")
        png = to_rgb_png(fetch_matplotlib(latex))
    path = OUT / f"{stem}.png"
    path.write_bytes(png)
    print("wrote", path.name, len(png), "bytes")


def main() -> None:
    for stem, latex in FORMULAS:
        render(stem, latex)


if __name__ == "__main__":
    main()
