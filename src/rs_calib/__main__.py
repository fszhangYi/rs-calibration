import sys


def _entry() -> int:
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
