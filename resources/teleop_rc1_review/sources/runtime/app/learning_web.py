"""Bounded same-origin API proxy for the separate loopback training broker."""
import asyncio
import json
from pathlib import Path
import re

import httpx
from fastapi import Request
from fastapi.responses import JSONResponse

from station_network import same_origin
UPSTREAM = "http://127.0.0.1:8794"
TOKEN_PATH = Path("/test/app/learning-bridge.token")
NO_STORE = {"Cache-Control": "no-store", "X-Content-Type-Options": "nosniff"}
GET_PATH = re.compile(r"(?:status|jobs/[a-f0-9]{32}/logs)\Z")
POST_PATH = re.compile(r"(?:plan|preflight|jobs/start|jobs/[a-f0-9]{32}/stop|capture/check|capture/episodes/(?:start|stop|success|failure))\Z")


def reply(code, error):
    return JSONResponse({"error": error, "ready": False}, status_code=code, headers=NO_STORE)


def install(app):
    if any(getattr(route, "path", "").startswith("/api/learning") for route in app.routes):
        raise RuntimeError("Learning routes already installed")
    slots = asyncio.Semaphore(4)

    @app.api_route("/api/learning/{path:path}", methods=["GET", "POST"])
    async def route(request: Request, path: str):
        post = request.method == "POST"
        if not (POST_PATH if post else GET_PATH).fullmatch(path) or request.query_params:
            return reply(404, "训练接口不存在")
        origins = request.headers.getlist("origin")
        if not same_origin(request, optional=True):
            return reply(403, "请从 4446 工作站操作")
        body = None
        if post:
            if not same_origin(request) or request.headers.getlist("x-openarm-learning") != ["1"]:
                return reply(403, "训练请求来源无效")
            if request.headers.get("content-type", "").split(";", 1)[0].strip() != "application/json":
                return reply(415, "需要 JSON 请求")
            lengths = request.headers.getlist("content-length")
            if request.headers.getlist("transfer-encoding") or len(lengths) != 1 or not lengths[0].isdecimal() or not 0 < int(lengths[0]) <= 4096:
                return reply(413, "训练请求过大或长度无效")
            chunks = bytearray()
            async def read_chunks():
                async for chunk in request.stream():
                    chunks.extend(chunk)
                    if len(chunks) > 4096:
                        raise ValueError("request too large")
            try:
                await asyncio.wait_for(read_chunks(), timeout=2)
            except asyncio.TimeoutError:
                return reply(408, "读取请求超时")
            except ValueError:
                return reply(413, "训练请求过大")
            if len(chunks) != int(lengths[0]):
                return reply(400, "训练请求不完整")
            body = bytes(chunks)
        if slots.locked():
            return reply(429, "训练管理接口繁忙，请稍后重试")
        await slots.acquire()
        try:
            token = TOKEN_PATH.read_text().strip()
            async with httpx.AsyncClient(timeout=httpx.Timeout(12), trust_env=False, follow_redirects=False) as client:
                async with client.stream(request.method, UPSTREAM + "/" + path, content=body,
                                         headers={"Content-Type": "application/json", "Accept-Encoding": "identity",
                                                  "X-OpenArm-Learning-Bridge": token}) as response:
                    data = bytearray()
                    async for chunk in response.aiter_bytes():
                        data.extend(chunk)
                        if len(data) > 128 * 1024:
                            return reply(502, "训练服务响应过大")
                    if response.headers.get("content-type", "").split(";", 1)[0] != "application/json":
                        return reply(502, "训练服务响应无效")
                    value = json.loads(data)
                    if type(value) is not dict:
                        return reply(502, "训练服务响应无效")
                    return JSONResponse(value, status_code=response.status_code, headers=NO_STORE)
        except (OSError, ValueError, httpx.HTTPError):
            return reply(503, "训练管理服务暂不可达")
        finally:
            slots.release()
