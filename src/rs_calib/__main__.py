import sys


def _ensure_opencv() -> None:
    """OpenCV 5 wheels often omit calibrateHandEye — auto-repair to 4.x."""
    try:
        from rs_calib.infra.ensure_opencv import ensure_opencv_handeye

        st = ensure_opencv_handeye(repair=True)
        print(f"[rs-calib] {st.get('message')}", flush=True)
        if not st.get("ok"):
            print(
                "[rs-calib] 警告: OpenCV HandEye 仍不可用，双算法将回退为仅原生。"
                ' 可手动: pip install "opencv-python>=4.8,<5"',
                file=sys.stderr,
                flush=True,
            )
    except Exception as e:  # noqa: BLE001
        print(f"[rs-calib] OpenCV 自检失败: {e}", file=sys.stderr, flush=True)


def _entry() -> int:
    _ensure_opencv()
    if len(sys.argv) > 1 and sys.argv[1] in {"web", "--web"}:
        from rs_calib.web.server import main as web_main

        host = "0.0.0.0"
        port = 6006
        args = sys.argv[2:]
        if "--host" in args:
            host = args[args.index("--host") + 1]
        if "--port" in args:
            port = int(args[args.index("--port") + 1])
        web_main(host=host, port=port)
        return 0
    from rs_calib.app import main

    return main()


if __name__ == "__main__":
    raise SystemExit(_entry())
