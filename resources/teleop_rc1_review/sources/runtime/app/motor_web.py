# 项目中文注释（2026-10-29）：同源电机 API 适配。
# 调用与职责：把 /api/motors 请求交给原 host 队列；同源校验后处理显式网页操作。
# 运行边界：POST 可触发初始化、使能或关闭；GET 只读。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Same-origin public API, registered before TeleopXR's static catch-all."""
import asyncio
import json
import urllib.error
import urllib.request
from fastapi import Request
from fastapi.responses import JSONResponse


# 注释：复用 console_gateway.panel_proxy 访问内部 8781 队列；修改成功后再读取完整状态。
def proxy(path, payload=None):
    # Reuse the existing host queue/lifecycle; only adapt the same-origin API.
    from console_gateway import panel_proxy
    try:
        code,raw=panel_proxy('/'+path,None if payload is None else json.dumps(payload).encode())
        if payload is not None and code<300:
            state_code,raw=panel_proxy('/state')
            if state_code>=400:code=state_code
        return json.loads(raw),code
    except Exception:
        return {"state":"unknown","busy":False,"error":"电机控制服务未连接"},503


# 注释：在静态文件 catch-all 前注册同源电机与相机路由，避免请求落到静态 404。
def install(app):
    from camera_web import install as install_camera_api
    install_camera_api(app)
    # 注释：拒绝不同源请求和非 JSON 修改请求，保持原网页手动操作边界。
    async def validate(request):
        origin = request.headers.get("origin")
        if origin and origin != str(request.base_url).rstrip("/"):
            return JSONResponse({"error": "必须从机器人网页操作"}, status_code=403)
        if not request.headers.get("content-type", "").startswith("application/json"):
            return JSONResponse({"error": "需要JSON请求"}, status_code=415)
        return None

    # 注释：处理显式初始化 POST，校验请求体大小后交给原内部队列。
    @app.post("/api/motors/initialize")
    async def initialize(request: Request):
        invalid = await validate(request)
        if invalid is not None:
            return invalid
        body = await request.body()
        if not 0 < len(body) <= 256:
            return JSONResponse({"error": "请求大小无效"}, status_code=400)
        data, code = await asyncio.to_thread(proxy, "initialize", {})
        return JSONResponse(data, status_code=code, headers={"Cache-Control": "no-store"})

    # 注释：处理只读电机 GET，不调用使能动作。
    @app.get("/api/motors")
    async def state():
        data, code = await asyncio.to_thread(proxy, "state")
        return JSONResponse(data, status_code=code, headers={"Cache-Control": "no-store"})

    @app.post("/api/motors/park")
    async def park(request: Request):
        """页面正常关闭：先收臂，再关闭使能；立即关闭仍使用原 enabled:false。"""
        invalid = await validate(request)
        if invalid is not None: return invalid
        body = await request.body()
        if not 0 < len(body) <= 256:
            return JSONResponse({"error": "请求大小无效"}, status_code=400)
        try:
            payload = json.loads(body)
            if (not isinstance(payload, dict) or set(payload) != {"request_id"}
                    or not isinstance(payload['request_id'], str)):
                raise ValueError()
        except Exception:
            return JSONResponse({"error": "需要唯一 request_id"}, status_code=400)
        data, code = await asyncio.to_thread(proxy, "set", {"park": True, **payload})
        return JSONResponse(data, status_code=code, headers={"Cache-Control": "no-store"})

    # 注释：只接受 enabled 布尔值及可选 request_id，拒绝额外字段后提交原开关动作。
    @app.post("/api/motors")
    async def set_state(request: Request):
        invalid = await validate(request)
        if invalid is not None:
            return invalid
        body = await request.body()
        if not 0 < len(body) <= 256:
            return JSONResponse({"error": "请求大小无效"}, status_code=400)
        try:
            payload = json.loads(body)
            if (not isinstance(payload,dict) or type(payload.get("enabled")) is not bool
                    or set(payload)-{"enabled","request_id"}):
                raise ValueError()
        except Exception:
            return JSONResponse({"error": "enabled必须为布尔值"}, status_code=400)
        data, code = await asyncio.to_thread(proxy, "set", payload)
        return JSONResponse(data, status_code=code, headers={"Cache-Control": "no-store"})
