# 项目中文注释（2026-10-29）：统一工作站接口登记。
# 调用与职责：为 4446 注册接口清单、UMI 接入状态及未接入后端的 503 占位。
# 运行边界：不创建 ROS、设备或电机操作；UMI 状态不是硬件快照。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Same-origin station discovery and explicit reservations for paused UMI.

No device, ROS, control publisher, subprocess or enable action.
Only bounded GET-only UMI diagnostic requests are delegated to the reviewed adapter.
The real UMI backend must replace these reservations during its integration.
"""
from fastapi.responses import JSONResponse
from umi_readonly_web import diagnostics as umi_diagnostics, session as umi_session, camera as umi_camera, status as umi_status_snapshot

SCHEMA = 'oa.station.interfaces.v1'
from station_network import preferred_origin
HEADERS = {'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff'}


# 注释：返回版本化的工作站入口、接口及当前控制集成边界。
def interfaces():
    return {
        'schema': SCHEMA,
        'revision': '20261103.teleop-capture1',
        'canonical_origin': preferred_origin(),
        'frontend': {'overview': '/#overview', 'vr': '/#vr', 'umi': '/#umi', 'learning': '/#learning'},
        'api': {
            'discovery': '/api/station/interfaces',
            'learning': {'path': '/api/learning/status', 'frontend': '/#learning',
                         'methods': ['GET'], 'local_profiles': ['act_local', 'smolvla_local'],
                         'plan': '/api/learning/plan', 'preflight': '/api/learning/preflight',
                         'start': '/api/learning/jobs/start', 'server_execution': False},
            'motors': {'path': '/api/motors', 'methods': ['GET', 'POST'],
                       'owner': 'shared_existing_manual_enable'},
            'motor_initialize': {'path': '/api/motors/initialize', 'methods': ['POST']},
            'vr_diagnostics': {'path': '/api/teleop/diagnostics', 'methods': ['GET']},
            'teleop_capture': {'status': '/api/capture/teleop/status',
                'action': '/api/capture/teleop/action',
                'records': '/api/capture/teleop/{vr|umi}/records',
                'modes': ['vr', 'umi'], 'training_eligible': False,
                'vr_keys': {'start': 'left_Y', 'stop': 'right_B'}},
            'vr_input': {'path': '/ws', 'transport': 'existing_websocket'},
            'umi_status': {'path': '/api/umi/status', 'methods': ['GET'],
                           'scope': 'installed_motion_adapter_current_status'},
            'umi_diagnostics': {'path': '/api/umi/diagnostics', 'methods': ['GET'],
                                'availability': 'read_only_diagnostic_proxy'},
            'umi_motion': {'status': '/api/umi/motion/status', 'action': '/api/umi/motion/action',
                           'source_default': 'none', 'motor_enable': 'existing_manual_webpage_only'},
            'umi_session': {'path': '/api/umi/session', 'methods': ['GET'],
                            'availability': 'read_only_diagnostic_proxy'},
            'umi_actions': {'paths': ['/api/umi/session/prepare', '/api/umi/session/start',
                                     '/api/umi/session/stop'], 'methods': ['POST'],
                            'availability': 'reserved_returns_503_no_forwarding'},
        },
        'control': {
            'ik': 'teleop-xr==1.3.5',
            'enable': 'existing_manual_webpage_only',
            'reset': 'existing_A_right_X_left',
            'single_can_writer_required': True,
            'right_can': 'can0', 'left_can': 'can1',
            'umi_motion_connected': True,
            'control_source_switch': 'explicit_none_vr_umi',
            'web_available_without_can': True,
            'hardware_reconnect_requires_manual_restart': True,
        },
        'compatibility_origins': [],
        'retired_ports': [4443, 4445, 4447, 4448],
        'umi_backend': 'existing_ik_candidate_ingress',
        'notes': ['Use relative URLs in frontend modules.',
                  'Internal media/diagnostic service ports are not workstation entry URLs.',
                  'This discovery document grants no motion permission.'],
    }


# 注释：返回明确的 503，说明真实 UMI 后端尚未接入。
def unavailable():
    return JSONResponse({
        'error': 'umi_backend_not_connected',
        'message': 'UMI 后端尚未接入；请使用现有共用双臂控制，勿另启 CAN 驱动。',
        'available': False, 'robot_commands': False,
        'real_control_ready': False, 'training_eligible': False,
    }, status_code=503, headers=HEADERS)


# 注释：先拒绝重复路由，再注册接口发现与明确不可用的 UMI 后端占位；不挂接设备。
def install(app):
    paths = {'/api/umi/cameras/{serial}/{eye}.jpg', '/api/station/interfaces', '/api/umi/status', '/api/umi/diagnostics',
             '/api/umi/session', '/api/umi/session/prepare', '/api/umi/session/start',
             '/api/umi/session/stop'}
    if paths.intersection(getattr(route, 'path', '') for route in app.routes):
        raise RuntimeError('Station/UMI route already installed; review integration instead of shadowing it')

    # 注释：提供 GET /api/station/interfaces 的只读接口清单。
    @app.get('/api/station/interfaces')
    async def discovery():
        return JSONResponse(interfaces(), headers=HEADERS)

    # 注释：仅报告 UMI 的 Web 集成状态，不合成设备或训练数据。
    app.add_api_route('/api/umi/status', umi_status_snapshot, methods=['GET'])
    app.add_api_route('/api/umi/diagnostics', umi_diagnostics, methods=['GET'])
    app.add_api_route('/api/umi/session', umi_session, methods=['GET'])
    app.add_api_route('/api/umi/cameras/{serial}/{eye}.jpg', umi_camera, methods=['GET'])
    for name in ('prepare', 'start', 'stop'):
        app.add_api_route('/api/umi/session/'+name, unavailable, methods=['POST'])
    from learning_web import install as install_learning_web
    install_learning_web(app)
