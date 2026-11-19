"""Independent loopback HTTP service for TXT piano performance."""
import argparse
import webbrowser

from paino_skill.controller import create_server


def main():
    parser = argparse.ArgumentParser(description="OpenArm TXT piano performance service")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--sim-port", type=int, default=8769)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    server = create_server(args.port, args.sim_port, score_only=True)
    url = f"http://127.0.0.1:{server.server_port}/"
    print(url, flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
