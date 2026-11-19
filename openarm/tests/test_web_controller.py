import json
import threading
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
import pytest
from openarm_app.web_controller import create_web_server


@pytest.fixture
def web(monkeypatch):
    calls = []
    class Client:
        def __init__(self, *args, **kwargs):
            pass
        def state(self):
            return {"api_version": 2, "state": "IDLE"}
        def command(self, action, **values):
            calls.append((action, values))
            return {"state": "EXECUTING"}
    monkeypatch.setattr("openarm_app.web_controller.SimClient", Client)
    server = create_web_server(0)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    yield f"http://127.0.0.1:{server.server_port}", calls
    server.shutdown()
    server.server_close()
    worker.join()


def fetch(url, payload=None, headers=None):
    return build_opener(ProxyHandler({})).open(Request(url, data=payload, headers=headers or {}), timeout=5)


def test_html_assets_and_same_origin_dual_commands(web):
    url, calls = web
    for path, mime in (("/", "text/html"), ("/app.js", "text/javascript"), ("/style.css", "text/css")):
        with fetch(url+path) as response:
            assert mime in response.headers["Content-Type"]
            assert response.read()
    with fetch(url+"/api/state") as response:
        assert json.load(response)["api_version"] == 2
    q = {"left":[0]*7, "right":[.1]*7}
    with fetch(url+"/api/command", json.dumps({"action":"move_joints", "arm":"both", "q":q}).encode(),
               {"Content-Type":"application/json", "Origin":url}) as response:
        assert response.status == 200
    assert calls == [("move_joints", {"arm":"both", "q":q})]


@pytest.mark.parametrize("path,body,headers,status", [
    ("/api/command", b'{"action":"home"}', {"Origin":"https://example.com", "Content-Type":"application/json"}, 403),
    ("/api/command", b'{"action":"home"}', {"Content-Type":"text/plain"}, 403),
    ("/api/command", b'{"action":"enable_motor"}', {"Content-Type":"application/json"}, 400),
    ("/api/command", b'[]', {"Content-Type":"application/json"}, 400),
    ("/api/command", b'x'*16385, {"Content-Type":"application/json"}, 400),
    ("/../AGENTS.md", None, {}, 404),
    ("/", None, {"Host":"example.com"}, 403),
])
def test_web_rejects_untrusted_and_invalid_requests(web, path, body, headers, status):
    url, calls = web
    with pytest.raises(HTTPError) as error:
        fetch(url+path, body, headers)
    assert error.value.code == status
    assert calls == []
