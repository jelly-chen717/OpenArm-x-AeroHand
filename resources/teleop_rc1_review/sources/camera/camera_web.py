"""Bounded asynchronous same-origin proxy for three read-only camera previews.

Install before TeleopXR's static catch-all. The only upstream is fixed HTTP
loopback: this module does not open a camera, publish ROS, or call control APIs.
Device serial bindings below match this deployment's separately verified host
configuration; replacing devices requires an explicit configuration update.
"""
import asyncio
import json
import math
import ssl

import httpx
from fastapi import Request
from fastapi.responses import JSONResponse, Response


UPSTREAM_BASE = "http://127.0.0.1:8768"
UPSTREAM_TIMEOUT_S = 0.8
MAX_JPEG_BYTES = 512 * 1024
MAX_STATUS_BYTES = 8 * 1024
MAX_CONCURRENT_REQUESTS = 4
CAMERA_IDS = ("d435", "d405_1", "d405_2")
# Current SDK-confirmed configuration, not USB enumeration order. The host
# configuration must bind the same three distinct physical devices.
CAMERA_SERIALS = {
    "d435": "401522071685",
    "d405_1": "409122273067",
    "d405_2": "409122272257",
}
ALLOWED_PROFILES = frozenset(((320, 240), (640, 480)))
_NO_STORE = {"Cache-Control": "no-store"}
# HTTPX still initializes TLS configuration for an HTTP-only client. Construct
# the context once, not for every preview frame (no repeated CA parsing).
_SSL_CONTEXT = ssl.create_default_context()


class _ProtocolError(Exception):
    """An upstream response did not satisfy the finite preview contract."""


def _new_client():
    # At most four clients exist; each request closes its client/stream even on
    # timeout or cancellation. No proxy, redirect, or user-selected destination.
    return httpx.AsyncClient(
        verify=_SSL_CONTEXT,
        timeout=httpx.Timeout(UPSTREAM_TIMEOUT_S),
        limits=httpx.Limits(max_connections=1, max_keepalive_connections=0),
        follow_redirects=False,
        trust_env=False,
    )


def _unavailable(error, camera=None):
    return {
        "camera": camera,
        "available": False,
        "serial": None,
        "seq": -1,
        "age_ms": None,
        "capture_fps": 0.0,
        "preview_fps": 0.0,
        "width": 0,
        "height": 0,
        "error": error,
    }


def _error(code, message, camera=None):
    headers = dict(_NO_STORE)
    if code == 429:
        headers["Retry-After"] = "1"
    return JSONResponse(_unavailable(message, camera), status_code=code, headers=headers)


def _finite_nonnegative(value, optional=False):
    if optional and value is None:
        return None
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise _ProtocolError()
    return value


def _serial(value, camera, optional=False):
    if optional and value is None:
        return None
    if (type(value) is not str or not 1 <= len(value) <= 32
            or not value.isascii() or not value.isdigit()
            or value != CAMERA_SERIALS[camera]):
        raise _ProtocolError()
    return value


def _profile(width, height, optional=False):
    if type(width) is not int or type(height) is not int:
        raise _ProtocolError()
    if optional and (width, height) == (0, 0):
        return
    if (width, height) not in ALLOWED_PROFILES:
        raise _ProtocolError()


def _status_payload(raw, camera):
    try:
        data = json.loads(raw)
        if (type(data) is not dict or data["camera"] != camera
                or type(data["available"]) is not bool):
            raise _ProtocolError()
        available = data["available"]
        serial = _serial(data["serial"], camera, optional=not available)
        seq = data["seq"]
        if type(seq) is not int or not -1 <= seq <= 9223372036854775807:
            raise _ProtocolError()
        age = _finite_nonnegative(data["age_ms"], optional=not available)
        capture_fps = _finite_nonnegative(data["capture_fps"])
        preview_fps = _finite_nonnegative(data["preview_fps"])
        width, height = data["width"], data["height"]
        _profile(width, height, optional=not available)
        if data["error"] is not None and type(data["error"]) is not str:
            raise _ProtocolError()
        if available and seq < 0:
            raise _ProtocolError()
        # Never relay arbitrary extra fields or raw host exceptions/paths.
        return {
            "camera": camera,
            "available": available,
            "serial": serial,
            "seq": seq,
            "age_ms": age,
            "capture_fps": capture_fps,
            "preview_fps": preview_fps,
            "width": width,
            "height": height,
            "error": None if available else "相机暂不可用",
        }
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
        raise _ProtocolError() from exc


def _frame_headers(response, camera):
    if response.headers.get("x-camera-id") != camera:
        raise _ProtocolError()
    result = {**_NO_STORE, "X-Camera-Id": camera}
    seq_raw = response.headers.get("x-frame-seq")
    age_raw = response.headers.get("x-frame-age-ms")
    if (seq_raw is None or not seq_raw.isascii() or not seq_raw.isdigit()
            or len(seq_raw) > 19 or int(seq_raw) > 9223372036854775807):
        raise _ProtocolError()
    try:
        age = _finite_nonnegative(float(age_raw))
    except (ValueError, TypeError, OverflowError) as exc:
        raise _ProtocolError() from exc
    result["X-Frame-Seq"] = str(int(seq_raw))
    result["X-Frame-Age-Ms"] = format(age, ".12g")
    result["X-Camera-Serial"] = _serial(response.headers.get("x-camera-serial"), camera)
    values = [response.headers.get("x-frame-" + name) for name in ("width", "height")]
    if any(v is None or not v.isascii() or not v.isdigit() or len(v) > 4 for v in values):
        raise _ProtocolError()
    width, height = map(int, values)
    _profile(width, height)
    result["X-Frame-Width"] = str(width)
    result["X-Frame-Height"] = str(height)
    return result


async def _read_bounded(response, limit):
    # Reject compressed responses instead of unbounded decompression.
    encoding = response.headers.get("content-encoding", "identity").lower().strip()
    if encoding != "identity":
        raise _ProtocolError()
    declared = response.headers.get("content-length")
    if declared is not None:
        if (not declared.isascii() or not declared.isdigit()
                or len(declared) > 10 or int(declared) > limit):
            raise _ProtocolError()
        declared = int(declared)
    body = bytearray()
    async for chunk in response.aiter_bytes():
        if len(body) + len(chunk) > limit:
            raise _ProtocolError()
        body.extend(chunk)
    if declared is not None and len(body) != declared:
        raise _ProtocolError()
    return bytes(body)


async def _fetch(endpoint, camera, after=None):
    params = {"camera": camera}
    if after is not None:
        params["after"] = str(after)
    async with _new_client() as client:
        async with client.stream(
            "GET", UPSTREAM_BASE + "/" + endpoint, params=params,
            headers={"Accept-Encoding": "identity"},
        ) as upstream:
            if upstream.status_code == 503:
                return _error(503, "相机暂不可用", camera)
            if endpoint == "snapshot.jpg" and upstream.status_code == 304:
                return Response(status_code=304, headers=_frame_headers(upstream, camera))
            if upstream.status_code != 200:
                raise _ProtocolError()
            content_type = upstream.headers.get("content-type", "").split(";", 1)[0].lower().strip()
            if endpoint == "status":
                if content_type != "application/json":
                    raise _ProtocolError()
                body = await _read_bounded(upstream, MAX_STATUS_BYTES)
                return JSONResponse(_status_payload(body, camera), headers=_NO_STORE)
            if content_type != "image/jpeg":
                raise _ProtocolError()
            headers = _frame_headers(upstream, camera)
            body = await _read_bounded(upstream, MAX_JPEG_BYTES)
            # JPEG envelope only: no decoder computation on the XR event loop.
            if len(body) < 5 or not body.startswith(b"\xff\xd8\xff") or not body.endswith(b"\xff\xd9"):
                raise _ProtocolError()
            return Response(body, media_type="image/jpeg", headers=headers)


def _parse_query(request, snapshot=False):
    allowed = {"camera", "after"} if snapshot else {"camera"}
    query = request.query_params
    if any(key not in allowed or len(query.getlist(key)) != 1 for key in query):
        raise ValueError()
    camera = query.get("camera", "d435")
    if camera not in CAMERA_IDS:
        raise ValueError()
    if not snapshot:
        return camera, None
    raw = query.get("after", "-1")
    if raw == "-1":
        return camera, -1
    if (not raw.isascii() or not raw.isdigit() or len(raw) > 19
            or int(raw) > 9223372036854775807):
        raise ValueError()
    return camera, int(raw)


async def _watch_disconnect(request):
    # GET endpoints do not consume a request body. Await ASGI events, without
    # polling: browser AbortController does not automatically cancel a handler.
    while True:
        if (await request.receive())["type"] == "http.disconnect":
            return


async def _fetch_connected(request, endpoint, camera, after):
    fetch = asyncio.create_task(_fetch(endpoint, camera, after))
    disconnected = asyncio.create_task(_watch_disconnect(request))
    try:
        # One deadline covers connect/header/body, not one timeout per chunk.
        # It only cancels the fetch once; a second timer must not interrupt close.
        done, _ = await asyncio.wait(
            (fetch, disconnected), timeout=UPSTREAM_TIMEOUT_S,
            return_when=asyncio.FIRST_COMPLETED,
        )
        if not done:
            raise asyncio.TimeoutError()
        if disconnected in done:
            # The browser is already gone. Returning a finite sanitized response
            # avoids an exception while finally closes the upstream operation.
            await disconnected
            return _error(499, "相机预览请求已取消", camera)
        return await fetch
    finally:
        for task in (fetch, disconnected):
            if not task.done():
                task.cancel()
        await asyncio.gather(fetch, disconnected, return_exceptions=True)


def install(app):
    """Register only the two GET camera routes, before the static mount."""
    if (set(CAMERA_SERIALS) != set(CAMERA_IDS)
            or len(set(CAMERA_SERIALS.values())) != len(CAMERA_IDS)):
        raise ValueError("Camera serial bindings must be complete and distinct")
    for camera in CAMERA_IDS:
        _serial(CAMERA_SERIALS[camera], camera)
    slots = asyncio.Semaphore(MAX_CONCURRENT_REQUESTS)

    async def limited_fetch(request, endpoint, camera, after=None):
        # Positive acquire does not yield on one ASGI event loop. Four slots
        # shared by every camera/route; full immediately returns 429, no queue.
        if slots.locked():
            return _error(429, "相机预览请求过多", camera)
        await slots.acquire()
        try:
            # HTTPX limits each I/O; _fetch_connected limits the entire fetch.
            # Finish resource cleanup before reusing its global slot.
            return await _fetch_connected(request, endpoint, camera, after)
        except _ProtocolError:
            return _error(502, "相机服务响应无效", camera)
        except (httpx.HTTPError, asyncio.TimeoutError):
            return _error(503, "相机服务暂未连接", camera)
        finally:
            slots.release()

    @app.get("/api/camera/status")
    async def camera_status(request: Request):
        try:
            camera, _ = _parse_query(request)
        except ValueError:
            return _error(400, "相机查询参数无效")
        return await limited_fetch(request, "status", camera)

    @app.get("/api/camera/snapshot.jpg")
    async def camera_snapshot(request: Request):
        try:
            camera, after = _parse_query(request, snapshot=True)
        except ValueError:
            return _error(400, "相机查询参数无效")
        return await limited_fetch(request, "snapshot.jpg", camera, after)
