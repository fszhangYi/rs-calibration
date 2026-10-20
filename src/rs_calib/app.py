from __future__ import annotations

import logging
import sys
from pathlib import Path


def _ensure_paths() -> None:
    root = Path(__file__).resolve().parents[2]
    src = root / "src"
    vendor = root / "vendor"
    for p in (str(src), str(vendor)):
        if p not in sys.path:
            sys.path.insert(0, p)


def main(argv: list[str] | None = None) -> int:
    _ensure_paths()
    from rs_calib.config.store import load_config
    from rs_calib.qt_compat import QApplication
    from rs_calib.ui.main_window import MainWindow

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    cfg = load_config()
    Path(cfg.app.session_root).mkdir(parents=True, exist_ok=True)
    Path("logs").mkdir(parents=True, exist_ok=True)

    app = QApplication(sys.argv if argv is None else argv)
    win = MainWindow(cfg)
    win.show()
    return app.exec_() if hasattr(app, "exec_") else app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
