import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

import numpy as np
import pytest

from openarm_app.kinematics import OpenArmIK
from openarm_app.kinematics.openarm_ik import pose_error
from paino_skill import controller
from paino_skill.points import load_table


def test_exported_points_match_calibration_and_fk():
    table = load_table()
    assert len(table["rows"]) == 58
    right = OpenArmIK()
    arms = {"right": right, "left": OpenArmIK(right.xml, side="left", model=right.setup.model)}
    for side, arm in arms.items():
        center = table["calibration"]["center_q_rad"][side]
        baseline = arm.forward(center)
        rows = [r for r in table["rows"] if r["arm"] == side]
        assert {r["offset_steps"] for r in rows} == set(range(-14,15))
        assert next(r["q"] for r in rows if r["offset_steps"] == 0) == center
        for row in rows:
            assert row["delta_y_m"] == pytest.approx(-row["offset_steps"]*.0235)
            if row["status"] != "ready":
                assert "q" not in row
                continue
            q = np.array(row["q"])
            assert np.all(q >= arm.limits[:,0]) and np.all(q <= arm.limits[:,1])
            target = baseline.copy()
            target[1] += row["delta_y_m"]
            pe, oe = pose_error(arm.forward(q), target)
            assert pe <= .0002 and oe <= np.deg2rad(.1)


def test_regions_and_unavailable_targets():
    table = load_table()
    for key in range(5,20):
        assert controller.key_target(table, key)[0] == "left"
    for key in range(24,39):
        assert controller.key_target(table, key)[0] == "right"
    for key in (1,4,20,21,22,23,39,53,True,24.0,"24"):
        with pytest.raises(ValueError):
            controller.key_target(table,key)


def test_http_click_routing_busy_guard_and_stop(monkeypatch):
    calls = []
    state = {"api_version":2,"scene":"piano","mode":"ideal","state":"IDLE"}
    class Client:
        def __init__(self,*args,**kwargs): pass
        def state(self): return state
        def piano_state(self,after): return {"last_event_id":after,"events":[]}
        def command(self,action,**kwargs):
            calls.append((action,kwargs))
            return {"state":"EXECUTING"}
    monkeypatch.setattr(controller,"SimClient",Client)
    server = controller.create_server(0)
    thread = threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    opener = build_opener(ProxyHandler({}))
    def post(payload,origin=url):
        return opener.open(Request(url+"/api/move",data=json.dumps(payload).encode(),
            headers={"Content-Type":"application/json","Origin":origin}),timeout=5)
    try:
        for path in ("/","/app.js","/audio.js","/style.css","/api/table","/api/timing","/api/piano?after=0","/points.csv"):
            with opener.open(url+path,timeout=5) as response: assert response.read()
        with opener.open(url+"/api/keyboard",timeout=5) as response:
            keys=json.load(response)["keys"]
        assert len(keys)==88 and sum(k["white"] for k in keys)==52
        assert keys[0]["midi"]==21 and keys[-1]["midi"]==108
        assert next(k for k in keys if k["midi"]==60)["white_key"]==24
        for midi in (21,60,108):
            with opener.open(url+f"/samples/{midi}.wav",timeout=5) as response:
                assert response.headers.get_content_type()=="audio/wav"
                assert response.read()==(controller.PIANO/"samples"/f"tone ({midi-20}).wav").read_bytes()
        for path in ("/samples/20.wav","/samples/109.wav","/samples/../scene.json"):
            with pytest.raises(HTTPError) as error: opener.open(url+path,timeout=5)
            assert error.value.code==404
        with post({"action":"key","key":25}): pass
        assert calls[-1] == ("move_joints",{"arm":"right","q":controller.key_target(load_table(),25)[1],"interpolation_steps":120,
                                             "velocity_limits":[2.]*7,"acceleration_limits":[20.]*7})
        with post({"action":"key","key":18,"interpolation_steps":120,"velocity_limit":1.5,"acceleration_limit":12}): pass
        assert calls[-1][1]["arm"] == "left"
        assert calls[-1][1]["interpolation_steps"] == 120
        assert calls[-1][1]["velocity_limits"] == [1.5]*7
        assert calls[-1][1]["acceleration_limits"] == [12.]*7
        with post({"action":"center","interpolation_steps":180}): pass
        assert calls[-1][1]["arm"] == "both"
        assert calls[-1][1]["interpolation_steps"] == 180
        assert calls[-1][1]["velocity_limits"] == [2.]*14
        assert calls[-1][1]["acceleration_limits"] == [20.]*14
        with post({"action":"play","midi":61,"arm":"right","finger":"middle","interpolation_steps":180}): pass
        assert calls[-1] == ("piano_play", {"arm":"right","midi":61,"finger":"middle",
                                           "interpolation_steps":180,"velocity_limits":[2.]*7,"acceleration_limits":[20.]*7})
        with post({"action":"finger","arm":"left","finger":"index","amount":.2}): pass
        assert calls[-1][0]=="piano_finger"
        with post({"action":"release"}): pass
        assert calls[-1][0]=="piano_release"
        count = len(calls)
        for field in ("velocity_limit", "acceleration_limit"):
            for value in (None, True, 0, -1, "2", float("inf"), float("nan"), []):
                with pytest.raises(HTTPError): post({"action":"center",field:value})
        for steps in (None, True, 0, -1, 2.5, "120", 100001):
            with pytest.raises(HTTPError): post({"action":"center","interpolation_steps":steps})
        for payload in ({"action":"key","key":22},{"action":"key","key":True},{"action":"enable_motor"}):
            with pytest.raises(HTTPError): post(payload)
        state["state"]="EXECUTING"
        with pytest.raises(HTTPError): post({"action":"key","key":26})
        assert len(calls)==count
        with post({"action":"stop"}): pass
        assert calls[-1][0]=="stop"
        with pytest.raises(HTTPError): post({"action":"stop"},"https://example.com")
        state.update(state="IDLE",scene="default")
        with pytest.raises(HTTPError): post({"action":"center"})
    finally:
        server.shutdown();server.server_close();thread.join()


def test_score_service_is_independent_and_routes_are_separate(monkeypatch):
    calls = []
    class Client:
        def __init__(self, port, **kwargs): assert port == 8769
        def state(self): return dict(api_version=2, scene="piano", mode="ideal")
        def piano_state(self, after): return {"events": [], "last_event_id": after}
        def command(self, action, **kwargs):
            calls.append((action, kwargs))
            return {"ok": True}
    monkeypatch.setattr(controller, "SimClient", Client)
    debug = controller.create_server(0)
    score = controller.create_server(0, score_only=True)
    threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in (debug, score)]
    for t in threads: t.start()
    opener = build_opener(ProxyHandler({}))
    debug_url = f"http://127.0.0.1:{debug.server_port}"
    url = f"http://127.0.0.1:{score.server_port}"
    def post(base, path, payload, origin=None):
        return opener.open(Request(base+path, data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json", "Origin": origin or base}), timeout=5)
    try:
        for path in ("/score.html", "/score.js", "/score.css", "/scores/moon.txt"):
            with pytest.raises(HTTPError) as e: opener.open(debug_url+path)
            assert e.value.code == 404
        with pytest.raises(HTTPError) as e: post(debug_url, "/api/score", {"action": "parse"})
        assert e.value.code == 404
        debug.shutdown(); debug.server_close(); threads[0].join()
        # The TXT service remains usable after the debug HTTP server shuts down.
        for path in ("/", "/score.js", "/score.css", "/audio.js", "/scores/moon.txt", "/api/keyboard", "/api/piano", "/samples/60.wav"):
            with opener.open(url+path) as response: assert response.read()
        text = "left_arm_move(17,5)\nright_arm_move(28,5)\nsleep(5)\nhand(D,1x)\nsleep(.1)\nhand(U,1x)"
        with post(url, "/api/score", dict(action="parse", text=text, finger_count=4)) as response:
            parsed = json.load(response)
        assert parsed["duration_us"] == 5_100_000 and parsed["note_count"] == 2
        assert parsed["actions"][0]["time_us"] == parsed["actions"][1]["time_us"] == 0
        assert not calls
        with pytest.raises(HTTPError): post(url, "/api/score", dict(action="parse", text="sleep(1)"))
        with post(url, "/api/score", dict(action="prepare", text=text, finger_count=4)): pass
        assert calls[-1][0] == "piano_score_prepare"
        with post(url, "/api/score", dict(action="start", token="example")): pass
        assert calls[-1] == ("piano_score_start", {"token": "example"})
        with post(url, "/api/score", dict(action="start", token="example", rate=.5)): pass
        assert calls[-1] == ("piano_score_start", {"token": "example", "rate": .5})
        for action in ("pause", "resume"):
            with post(url, "/api/score", dict(action=action)): pass
            assert calls[-1] == ("piano_score_" + action, {})
        with post(url, "/api/score", dict(action="rate", rate=2.)): pass
        assert calls[-1] == ("piano_score_rate", {"rate": 2.})
        with post(url, "/api/score", dict(action="stop", token="example", reason="page_closed")): pass
        assert calls[-1] == ("piano_score_stop", {"token": "example", "reason": "page_closed"})
        with post(url, "/api/move", dict(action="stop")): pass
        assert calls[-1][0] == "stop"
        count = len(calls)
        with pytest.raises(HTTPError): post(url, "/api/move", dict(action="center"))
        with pytest.raises(HTTPError) as e: post(url, "/api/move", dict(action="stop"), debug_url)
        assert e.value.code == 403 and len(calls) == count
        for path in ("/api/model", "/api/state", "/api/table", "/points.csv", "/app.js", "/viewer.js"):
            with pytest.raises(HTTPError) as e: opener.open(url+path)
            assert e.value.code == 404
    finally:
        for s in (debug, score): s.shutdown(); s.server_close()
        for t in threads: t.join()


def test_music_catalog_rescans_and_only_reads_named_txt(monkeypatch, tmp_path):
    from urllib.parse import quote
    folder = tmp_path / "music_data"
    folder.mkdir()
    monkeypatch.setattr(controller, "MUSIC_DIR", folder)
    song = folder / "月亮 & 星星.txt"
    song.write_text("\ufeffsleep(0)", encoding="utf-8")
    (folder / "ignore.wav").write_bytes(b"test")
    (tmp_path / "secret.txt").write_text("private", encoding="utf-8")
    server = controller.create_server(0, score_only=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    opener = build_opener(ProxyHandler({}))
    def get(path):
        with opener.open(url+path, timeout=5) as response: return json.load(response)
    try:
        assert get('/api/music') == {"songs": [{"id": song.name, "name": "月亮 & 星星"}]}
        assert get('/api/music/text?id='+quote(song.name)) == {"name": "月亮 & 星星", "text": "sleep(0)"}
        for name in ("../secret.txt", "..\\secret.txt", str(tmp_path / "secret.txt"), "ignore.wav"):
            with pytest.raises(HTTPError) as error: get('/api/music/text?id='+quote(name))
            assert error.value.code == 404
        song.unlink()
        assert get('/api/music') == {"songs": []}
        (folder / "新增.TXT").write_text("sleep(1)", encoding="utf-8")
        assert get('/api/music')["songs"][0]["name"] == "新增"
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_high_speed_adjacent_key_minimum_uses_continuous_extrema():
    from openarm_app.trajectory import JointTrajectoryPlanner
    planner = JointTrajectoryPlanner([-10]*7, [10]*7)
    table = load_table()
    for side, key in (("right",25),("left",18)):
        args = dict(q_start=table["calibration"]["center_q_rad"][side],
                    q_goal=controller.key_target(table,key)[1], velocity_limits=[2]*7,
                    acceleration_limits=[20]*7, sample_period=1/60)
        with pytest.raises(ValueError, match="至少需要 8 步"):
            planner.plan(**args, interpolation_steps=7)
        tr = planner.plan(**args, interpolation_steps=8)
        assert tr.time[-1] == 8/60
        np.testing.assert_array_equal(tr.q[-1], args["q_goal"])


def test_score_root_serves_user_standalone_with_simulation_bridge(monkeypatch, tmp_path):
    frontend = tmp_path / 'piano_standalone.html'
    original = '<html><script id="assets" type="application/json">{"songs":[],"samples":{"60":"large-wav"}}</script><script type="module">const ui = true;</script></html>'
    frontend.write_text(original, encoding='utf-8')
    monkeypatch.setattr(controller, 'STANDALONE_UI', frontend)
    monkeypatch.setattr(controller, 'MUSIC_DIR', tmp_path)
    song = tmp_path / '槿（实琴分手·副歌纯旋律）.txt'
    score = '# C4 < B4\nright_arm_move(30, 2)\nsleep(2)'
    song.write_text(score, encoding='utf-8-sig')
    server = controller.create_server(0, score_only=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = build_opener(ProxyHandler({}))
    try:
        with opener.open(f'http://127.0.0.1:{server.server_port}/') as response:
            html = response.read().decode('utf-8')
            assert 'const ui = true;' in html
            assert 'pollSimulation' in html
            assert 'large-wav' not in html
            assert '"samples": {}' in html
            asset_tag = '<script id="assets" type="application/json">'
            catalog = json.loads(html.split(asset_tag, 1)[1].split('</script>', 1)[0])
            assert catalog['songs'] == [{'name': song.stem, 'text': score}]
            assert 'nonce=' in html
            assert 'nonce-' in response.headers['Content-Security-Policy']
        assert frontend.read_text(encoding='utf-8') == original
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
