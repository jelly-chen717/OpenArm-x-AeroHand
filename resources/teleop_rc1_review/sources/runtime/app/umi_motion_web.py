"""Bounded same-origin, explicit actions for the existing UMI panel."""
import asyncio
import json
import secrets

from fastapi import Request
from fastapi.responses import JSONResponse
from fastapi.routing import APIRoute

HEADERS = {'Cache-Control':'no-store', 'X-Content-Type-Options':'nosniff'}
from station_network import same_origin


def install(app, runtime):
    async def startup():
        # UMI can be used without ever opening a headset XR session.
        runtime.worker.set_teleop_loop(asyncio.get_running_loop())
    app.router.add_event_handler('startup', startup)
    async def status(request: Request):
        if not same_origin(request, optional=True):
            return JSONResponse({'error':'same_origin_required'}, status_code=403, headers=HEADERS)
        return JSONResponse(await asyncio.to_thread(runtime.status), headers=HEADERS)

    async def integration_status(request: Request):
        if not same_origin(request, optional=True):
            return JSONResponse({'error':'same_origin_required'}, status_code=403, headers=HEADERS)
        s = await asyncio.to_thread(runtime.status)
        ready = (s['input_source'] == 'umi' and s['feedback_ok'] and
                 s['transport']['healthy'] and not s['bridge_problem'] and
                 s['mapping']['ready'] and bool(s['adapter'] and s['adapter']['engaged']))
        return JSONResponse({'schema':'oa.station.umi_integration.v1',
            'scope':'installed_motion_adapter_current_status', 'frontend_mounted':True,
            'backend_connected':True, 'backend_state':'existing_ik_candidate_ingress',
            'hardware_state':None, 'motion_connected':True, 'real_control_ready':bool(ready),
            'physical_motion_verified':False, 'control_source_switch':'explicit_none_vr_umi',
            'actions_forwarded':False, 'motion_status':'/api/umi/motion/status',
            'input_source':s['input_source'], 'active':s['active'],
            'message':'85 单臂平移适配已接入；实时许可见 motion/status。'}, headers=HEADERS)

    async def action(request: Request):
        if (not same_origin(request) or
                not request.headers.get('content-type','').startswith('application/json') or
                not secrets.compare_digest(request.headers.get('x-umi-token',''), runtime.token)):
            return JSONResponse({'error':'same_origin_json_and_token_required'}, status_code=403, headers=HEADERS)
        body = await request.body()
        if not 0 < len(body) <= 1024:
            return JSONResponse({'error':'invalid_body_size'}, status_code=400, headers=HEADERS)
        try:
            data = json.loads(body)
            if not isinstance(data,dict) or set(data)-{'action','source','point'}:
                raise ValueError('invalid_action_fields')
            result = await asyncio.to_thread(runtime.action, data)
            return JSONResponse(result, headers=HEADERS)
        except (ValueError,TypeError,KeyError) as exc:
            return JSONResponse({'error':str(exc)}, status_code=409, headers=HEADERS)

    paths = {'/api/umi/motion/status', '/api/umi/motion/action'}
    if paths.intersection(getattr(r,'path','') for r in app.routes):
        raise RuntimeError('UMI motion routes already installed')
    # TeleopXR has mounted StaticFiles('/') already. Insert before that catchall.
    app.router.routes[:] = [r for r in app.router.routes if getattr(r,'path','') != '/api/umi/status']
    app.router.routes[0:0] = [APIRoute('/api/umi/motion/status', status, methods=['GET']),
                             APIRoute('/api/umi/motion/action', action, methods=['POST']),
                             APIRoute('/api/umi/status', integration_status, methods=['GET'])]
