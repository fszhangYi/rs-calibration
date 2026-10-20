"""PyInstaller entry shim for Wine: disable isolated subprocess hooks."""

from __future__ import annotations

import sys

# Wine often kills PyInstaller isolated discovery workers.
sys._pyi_isolated_subprocess = True  # type: ignore[attr-defined]

from PyInstaller.__main__ import run

if __name__ == "__main__":
    run()
