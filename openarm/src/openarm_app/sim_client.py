"""Dependency-free client for the local simulation server."""
import argparse
import json
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError, URLError


class SimClient:
    def __init__(self, port=8769, timeout=30):
        self.url = f"http://127.0.0.1:{int(port)}"
        self.timeout = timeout
        self.opener = build_opener(ProxyHandler({}))

    def _request(self, action=None, path=None):
        body = None if action is None else json.dumps(action, allow_nan=False).encode()
        request = Request(self.url+(path or ("/state" if action is None else "/command")),
                          data=body, headers={"Content-Type": "application/json"})
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                payload = json.load(response)
        except HTTPError as exc:
            payload = json.load(exc)
            raise RuntimeError(payload.get("error", str(exc))) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"Simulation connection failed: {exc}. Check server status before retrying a motion.") from exc
        return payload["result"]

    def state(self):
        return self._request()

    def visual_model(self):
        return self._request(path="/visual/model")

    def piano_state(self, after=0):
        return self._request(path=f"/piano/state?after={int(after)}")

    def command(self, action, **values):
        return self._request({"action": action, **values})


def main():
    parser = argparse.ArgumentParser(description="Control the local ideal simulation")
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument("--arm", choices=("right", "left", "both"), default="right",
                        help="both: pass 14 numbers, right seven then left seven")
    sub = parser.add_subparsers(dest="action", required=True)
    for action in ("state", "pause", "resume", "stop", "home"):
        sub.add_parser(action)
    sub.add_parser("move_joints").add_argument("q", nargs="+", type=float)
    for action in ("move_pose", "preview_pose"):
        sub.add_parser(action).add_argument("pose", nargs="+", type=float)
    args = parser.parse_args()
    client = SimClient(args.port)
    try:
        fields = {key: getattr(args, key) for key in ("q", "pose") if hasattr(args, key)}
        count = 14 if args.arm == "both" else 7
        for key, values in fields.items():
            if len(values) != count:
                raise ValueError(f"{args.arm} requires {count} values for {key}")
            if args.arm == "both":
                fields[key] = {"right": values[:7], "left": values[7:]}
        fields["arm"] = args.arm
        result = client.state() if args.action == "state" else client.command(args.action, **fields)
        print(json.dumps(result, indent=2))
    except (RuntimeError, ValueError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
