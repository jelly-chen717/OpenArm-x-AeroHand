# 项目中文注释（2026-10-29）：IK 与原运动桥许可事务。
# 调用与职责：验证 ROS 图、16 个使能反馈和实例身份；先开 IK，再开原桥，失败则尝试关闭二者。
# 运行边界：不直接生成运动目标，但 on 会允许后续 VR 与复位输出。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Explicit archive bridge ON/OFF/status; no motion or hardware commands.

The host panel owns serialization. The host wrapper never times out/kills an
ON helper that could have submitted True. ROS calls reuse the existing lifecycle
adapter's persistent clients; a failed ON attempts both False calls on those
same clients before it returns.
"""
import argparse
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import time
import uuid


HOST_ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
SOURCE_SHA256 = '1e5d17c0dcaecd2ce46e1005d878d750978f9930180b1c348719f739b2612ccb'
BRIDGE_STATE = '/archive_control/bridge_state'
IK_GATE = '/motor_toggle/ik_gate'
WORKER_READY_TIMEOUT_S = 60.0
TOPICS = tuple('/'+side+suffix for side in ('left', 'right') for suffix in
               ('_forward_position_controller/commands', '_gripper_controller/joint_trajectory'))
CONTROLLERS = {'joint_state_broadcaster', 'left_forward_position_controller', 'right_forward_position_controller',
               'left_gripper_controller', 'right_gripper_controller'}
HARDWARE = {'openarm_right_hardware_interface', 'openarm_left_hardware_interface'}
CANCEL_NAME = re.compile(r'(?:panel_cancel_[0-9a-f]{32}|panel_motor_cancel_[0-9a-f]{32}'
                         r'|hardware_cancel_[0-9a-f]{32}|hardware_cancel_[0-9a-f]{8}_[1-9][0-9]*)\.request\Z')


# 注释：条件不成立时立即拒绝本次操作，并保留可解释的错误原因。
def require(condition, message):
    if not condition: raise RuntimeError(message)


# 注释：判断输入是否为有限数值，避免 NaN/Inf 进入状态或控制判断。
def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


# 注释：原实现说明：The saved worker is still starting; no True request was submitted.
class WorkerReadinessTimeout(RuntimeError):
    """The saved worker is still starting; no True request was submitted."""


# 注释：把取消文件限制在固定 state 目录与白名单名称。
def cancel_path(value, inside=False):
    if value is None: return None
    text = str(value)
    path = PurePosixPath(text)
    base = PurePosixPath('/state' if inside else HOST_ROOT.as_posix())
    if not inside: base /= 'state'
    require('\\' not in text and '..' not in path.parts and path.parent == base
            and CANCEL_NAME.fullmatch(path.name), 'Cancel file is outside the fixed state whitelist')
    return Path(str(path))


# 注释：检查取消请求文件的路径、类型和所有者，再判断是否已请求取消。
def cancelled_file(path):
    if path is None: return False
    require(path.parent.resolve() == path.parent and not path.is_symlink(), 'Unsafe cancellation path')
    if not path.exists(): return False
    require(path.is_file() and path.stat().st_uid == os.geteuid(), 'Unsafe cancellation file owner/type')
    return True


# 注释：核对原桥、控制管理器、许可服务和命令话题的唯一所属者。
def validate_graph(nodes, services, publishers, require_ik=True):
    bridge = ('jt_bridge', '/')
    require(nodes.count(bridge) == 1 and nodes.count(('controller_manager', '/')) == 1,
            'One root bridge and one controller manager are required')
    require(not any(name == 'vr_custom_chain_bridge' for name, namespace in nodes),
            'The custom motion bridge is still present')
    for endpoint in ('/enable', BRIDGE_STATE):
        require(services.get(endpoint) == [bridge], 'Archive bridge service owner differs: '+endpoint)
    owners = services.get(IK_GATE, [])
    if require_ik or owners:
        require(len(owners) == 1 and owners[0] != bridge,
                'One original worker IK gate owner is required')
    for topic in TOPICS:
        require(publishers.get(topic) == [bridge], 'Duplicate or unexpected command writer: '+topic)


# 注释：核对原桥源码、实例、状态结构和新鲜度，可额外要求输出及复位已关闭。
def validate_state(state, now, instance=None, closed=False):
    require(isinstance(state, dict) and state.get('kind') == 'archive_control_bridge_state_v1'
            and state.get('source_sha256') == SOURCE_SHA256
            and state.get('source_module') == 'openarm_teleop_bridge.jt_bridge', 'Archive source proof differs')
    require(isinstance(state.get('instance_id'), str) and re.fullmatch('[0-9a-f]{32}', state['instance_id'])
            and (instance is None or state['instance_id'] == instance), 'Archive bridge instance changed')
    stamp = state.get('heartbeat_monotonic_s')
    require(finite(stamp) and 0 <= now-stamp <= .5, 'Archive bridge state is stale')
    sides = state.get('home_active_sides')
    require(type(state.get('bridge_enabled')) is bool and type(state.get('home_active')) is bool
            and isinstance(sides, list) and all(side in ('left', 'right') for side in sides)
            and len(set(sides)) == len(sides)
            and state['home_active'] == bool(sides), 'Archive output/home state is malformed')
    if closed:
        require(state['bridge_enabled'] is False and state['home_active'] is False
                and state.get('home_targets') == {'left': None, 'right': None}, 'Archive OFF did not clear output/home')
    return state


# 注释：核对 16 个健康使能电机、活动硬件组件和五个原控制器。
def hardware_hold(status):
    # Reuse the standard motor identity/state/freshness/finite-position checks.
    from hardware_lifecycle_20261020 import validate_can
    validate_can(status, {'left', 'right'})
    require(all(type(status.get(key)) is int and status[key] == expected for key, expected in
                (('online', 16), ('enabled', 16), ('faulted', 0))), 'Sixteen healthy enabled motors required')
    require(set(status.get('hardware_states', {})) == HARDWARE
            and set(status['hardware_states'].values()) == {3}, 'Hardware is not active hold')
    require(set(status.get('controllers', {})) == CONTROLLERS
            and set(status['controllers'].values()) == {'active'}, 'Five original active controllers required')
    for key in ('hardware_age_ms', 'controllers_age_ms'):
        require(finite(status.get(key)) and 0 <= status[key] < 5000, 'Hardware lifecycle observation is stale')


# 注释：分别尝试关闭 IK 和原桥，并回读复位/输出关闭结果。
def output_off(io):
    """Attempt both False calls even when tracking, health or one service fails."""
    errors, events = [], []
    for endpoint in (IK_GATE, '/enable'):
        try:
            io.gate(endpoint, False); events.append(endpoint+'_false_ack')
        except Exception as error: errors.append(endpoint+': '+str(error))
    try: io.graph()
    except Exception as error: errors.append('graph: '+str(error))
    state = None
    try: state = validate_state(io.state(), io.clock(), closed=True)
    except Exception as error: errors.append('readback: '+str(error))
    return dict(success=not errors, events=events, errors=errors, bridge=state)


# 注释：等待同一 IK 工作者就绪，同时持续确认原桥身份和电机保持。
def wait_worker_ready(io, cancelled, timeout=WORKER_READY_TIMEOUT_S):
    """Only observe startup; JAX constructs the gate before any XR is needed.

    Hardware-only enable restarts the runtime and can finish before the saved
    worker creates its IK service. Keep observing that same bridge and healthy
    motor hold while waiting, without submitting either True request.
    """
    require(finite(timeout) and 0 < timeout <= WORKER_READY_TIMEOUT_S, 'Invalid worker readiness bound')
    started = io.clock(); deadline = started+timeout; instance = None
    while True:
        require(not cancelled(), 'Archive ON cancelled during worker startup')
        io.graph(require_ik=False)
        state = validate_state(io.state(), io.clock(), instance)
        instance = state['instance_id']
        hardware_hold(io.hardware())
        require(not cancelled(), 'Archive ON cancelled during worker startup')
        ready = io.worker_ready()
        remaining = deadline-io.clock()
        if remaining < 0 or not ready and remaining == 0:
            raise WorkerReadinessTimeout('Saved worker IK service was not ready within '+str(timeout)+' seconds')
        if ready:
            io.graph()
            require(not cancelled(), 'Archive ON cancelled after worker startup')
            if io.clock() > deadline:
                raise WorkerReadinessTimeout('Saved worker IK service was not ready within '+str(timeout)+' seconds')
            return dict(waited_s=io.clock()-started, timeout_s=timeout, requires_xr=False)
        io.spin(min(.2, remaining))


# 注释：on 先等工作者，再依次开启 IK 与原桥并回读；off 分别尝试关闭；异常返回包含回退结果的回执。
def execute(action, io, cancelled=lambda: False):
    """Injectable transaction; the IO exposes only gates, state and observations."""
    require(action in ('on', 'off', 'status', 'park'), 'Unknown archive action')
    if action == 'park': return execute_park(io, cancelled)
    receipt = dict(kind='archive_output_receipt_v1', action=action, hardware_commands=False,
                   motion_targets_published=False)
    if action == 'off': return dict(receipt, **output_off(io))
    try:
        require(action != 'on' or not cancelled(), 'Archive ON cancelled')
        if action == 'on': receipt['worker_readiness'] = wait_worker_ready(io, cancelled)
        io.graph()
        state = validate_state(io.state(), io.clock())
        if action == 'status': return dict(receipt, success=True, bridge=state)
        instance = state['instance_id']
        hardware_hold(io.hardware())
        require(not cancelled(), 'Archive ON cancelled before IK')
        io.gate(IK_GATE, True)
        require(not cancelled(), 'Archive ON cancelled after IK')
        hardware_hold(io.hardware())
        validate_state(io.state(), io.clock(), instance)
        io.graph()
        require(not cancelled(), 'Archive ON cancelled before bridge')
        io.gate('/enable', True)
        require(not cancelled(), 'Archive ON cancelled after bridge')
        state = validate_state(io.state(), io.clock(), instance)
        require(state['bridge_enabled'] is True, 'Archive bridge True was not observed')
        hardware_hold(io.hardware())
        require(not cancelled(), 'Archive ON cancelled after verification')
        return dict(receipt, success=True, bridge=state, hardware_hold_verified=True,
                    events=['ik_true_ack', 'bridge_true_ack'])
    except BaseException as error:
        result = dict(receipt, success=False, error=type(error).__name__+': '+str(error))
        if isinstance(error, WorkerReadinessTimeout): result['worker_readiness_timeout'] = True
        if action == 'on': result['off_rollback'] = output_off(io)
        return result


def execute_park(io, cancelled=lambda: False):
    """原桥独占收臂；此事务只生成到位证明，关闭电机仍走原生命周期。"""
    receipt = dict(kind='archive_output_receipt_v1', action='park', hardware_commands=False,
                   motion_targets_published=False, motion_via_original_bridge=True)
    started = False
    try:
        require(not cancelled(), '收臂已取消')
        io.graph()
        io.park_graph()
        hardware_hold(io.hardware())
        initial = validate_state(io.state(), io.clock())
        instance = initial['instance_id']
        # 先停止 IK 与原桥，再由原桥的独占阶段器运动，避免手柄或 A/X 抢写。
        closed = output_off(io)
        require(closed['success'], '收臂前未能关闭遥操输出')
        validate_state(io.state(), io.clock(), instance, closed=True)
        hardware_hold(io.hardware())
        require(not cancelled(), '收臂已取消')
        started = True  # 即使服务超时也必须请求取消，不能认为它没有开始。
        state = io.park('start')
        validate_state(state, io.clock(), instance)
        park_id = state.get('parking', {}).get('id')
        require(isinstance(park_id, str) and re.fullmatch('[0-9a-f]{32}', park_id), '收臂事务身份缺失')
        deadline = io.clock()+100.
        while True:
            require(not cancelled(), '收臂已取消')
            hardware_hold(io.hardware())
            state = validate_state(io.park('heartbeat'), io.clock(), instance)
            park = state.get('parking') or {}
            require(park.get('id') == park_id and park.get('owned') is True, '收臂事务已更换')
            require(park.get('status') in ('running', 'complete'), park.get('error') or '收臂中止')
            require(state['bridge_enabled'] is True, '收臂输出许可已关闭')
            if park['status'] == 'complete':
                hardware_hold(io.hardware())
                require(not cancelled(), '收臂完成时已取消')
                # 完成状态保留占用，先关闭桥，才交给外层解除电机使能。
                closed = output_off(io)
                require(closed['success'], '垂下到位，但输出关闭未确认')
                return dict(receipt, success=True, parked=True, park_id=park_id,
                            bridge_instance=instance, final_stage=park['phase'],
                            final_target=park['target'], feedback_arrival_verified=True)
            require(io.clock() < deadline, '收臂总时长超时')
            io.spin(.05)
    except BaseException as error:
        result = dict(receipt, success=False, error=str(error), cancelled=bool(cancelled()), parked=False)
        if started:
            try: io.park('cancel')
            except Exception as stop_error: result['cancel_error'] = str(stop_error)
        result['off_rollback'] = output_off(io)
        return result


# 注释：原实现说明：Reuse existing lifecycle adapter clients/spin; never call its actuators.
class InsideIO:
    """Reuse existing lifecycle adapter clients/spin; never call its actuators."""
    # 注释：初始化 InsideIO 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self):
        import hardware_lifecycle_20261020 as lifecycle
        lifecycle.validate_domain(os.environ)
        import rclpy
        from std_srvs.srv import SetBool, Trigger
        self.rclpy, self.SetBool, self.Trigger = rclpy, SetBool, Trigger
        rclpy.init(); self.adapter = lifecycle.make_ros_adapter()
        self.park_identity = uuid.uuid4().hex

    clock = staticmethod(time.monotonic)

    def park_graph(self):
        bridge = ('jt_bridge', '/')
        nodes = self.adapter.node.get_node_names_and_namespaces()
        endpoint = '/archive_control/park_command'
        owners = [(name, ns) for name,ns in nodes for service,kinds in
                  self.adapter.node.get_service_names_and_types_by_node(name, ns)
                  if service == endpoint and kinds == ['rcl_interfaces/srv/SetParametersAtomically']]
        require(owners == [bridge], '收臂服务所属者不唯一: '+endpoint)

    def park(self, action):
        from rcl_interfaces.srv import SetParametersAtomically
        from rcl_interfaces.msg import Parameter, ParameterValue
        require(action in ('start', 'heartbeat', 'cancel'), 'Unknown park action')
        command = json.dumps({'id':self.park_identity, 'action':action, 'expires':self.clock()+.8})
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name='command', value=ParameterValue(type=4, string_value=command))]
        response = self.adapter.call('/archive_control/park_command', request, SetParametersAtomically)
        require(response.result.successful, response.result.reason)
        return json.loads(response.result.reason)

    # 注释：向既有 IK 或桥许可服务发送布尔值；只允许固定端点。
    def gate(self, endpoint, value):
        require(endpoint in (IK_GATE, '/enable') and type(value) is bool, 'Unknown archive gate')
        request = self.SetBool.Request(); request.data = value
        return self.adapter.call(endpoint, request, self.SetBool)

    # 注释：通过原桥只读状态服务获得 JSON，并限制响应长度。
    def state(self):
        response = self.adapter.call(BRIDGE_STATE, self.Trigger.Request(), self.Trigger)
        require(len(response.message) <= 65536, 'Archive bridge state is oversized')
        return json.loads(response.message)

    # 注释：读取生命周期适配器的硬件状态。
    def hardware(self): return self.adapter.status()

    # 注释：让 ROS 执行回调并等待指定时长。
    def spin(self, duration): self.adapter.spin(duration)

    # 注释：只发现 IK 服务是否出现，不发送开启请求。
    def worker_ready(self):
        # This is discovery only. Reuse the very same persistent client as call().
        if IK_GATE not in self.adapter.clients:
            self.adapter.clients[IK_GATE] = self.adapter.node.create_client(self.SetBool, IK_GATE)
        return self.adapter.clients[IK_GATE].wait_for_service(timeout_sec=0.0)

    # 注释：读取 ROS 图并核对控制发布者和服务归属。
    def graph(self, require_ik=True):
        self.adapter.assert_single_manager()
        deadline = time.monotonic()+3
        while True:
            self.adapter.spin(.05)
            try:
                node = self.adapter.node; nodes = node.get_node_names_and_namespaces()
                services = {endpoint: [] for endpoint in ('/enable', BRIDGE_STATE, IK_GATE)}
                for name, namespace in nodes:
                    for endpoint, kinds in node.get_service_names_and_types_by_node(name, namespace):
                        if endpoint in services:
                            required = 'std_srvs/srv/Trigger' if endpoint == BRIDGE_STATE else 'std_srvs/srv/SetBool'
                            require(kinds == [required], 'Archive service type differs')
                            services[endpoint].append((name, namespace))
                publishers = {topic: [(info.node_name, info.node_namespace)
                                     for info in node.get_publishers_info_by_topic(topic)] for topic in TOPICS}
                validate_graph(nodes, services, publishers, require_ik=require_ik); return
            except Exception:
                if time.monotonic() >= deadline: raise

    # 注释：释放当前对象持有的线程、ROS 或网络资源。
    def close(self):
        self.adapter.node.destroy_node()
        if self.rclpy.ok(): self.rclpy.shutdown()


# 注释：验证原所属容器后通过 docker exec 调用已有容器内许可事务，继承操作锁且不另建 CAN 驱动。
def host(action, cancel=None, manager=None, run=subprocess.run):
    if manager is None:
        import manage as manager
    settings = manager.runtime(); container = manager.original_container(settings)
    require(container['State']['Running'], 'Owned archive runtime is stopped')
    manager.backup_complete(settings); manager.check_owned_dropin(settings)
    require(not manager.customized_container()['State']['Running'], 'Customized backup is running')
    manager.require_tests_stopped(); manager.require_no_other_managers(container)
    args = [action]
    if cancel is not None: args += ['--cancel-file', '/state/'+cancel.name]
    shell = '''set -eo pipefail
source /opt/ros/humble/setup.bash
source /test/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export HOME=/tmp/original-vr-home ROS_LOG_DIR=/logs/ros
exec /usr/bin/python3 -B /test/ops/archive_output.py --inside "$@"
'''
    command = ['docker', 'exec', '--user', '1002:1002', settings['container_id'], '/usr/bin/env', '-i',
               'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin', 'LANG=C.UTF-8',
               '/bin/bash', '-c', shell, 'archive-output', *args]
    from operation_lock import child_fds
    result = run(command, capture_output=True, text=True, timeout=None, pass_fds=child_fds())
    lines = [line[len('ARCHIVE_OUTPUT_RECEIPT='):] for line in result.stdout.splitlines()
             if line.startswith('ARCHIVE_OUTPUT_RECEIPT=')]
    require(len(lines) == 1, 'Archive helper returned no unique receipt: '+result.stderr[-800:])
    receipt = json.loads(lines[0])
    require(isinstance(receipt, dict) and receipt.get('kind') == 'archive_output_receipt_v1'
            and receipt.get('action') == action, 'Archive helper receipt differs')
    require(not result.returncode or receipt.get('success') is not True, 'Archive helper exit and receipt disagree')
    return receipt


# 注释：该文件的显式执行入口；验证 ROS 图、16 个使能反馈和实例身份；先开 IK，再开原桥，失败则尝试关闭二者。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('on', 'off', 'status', 'park'))
    parser.add_argument('--inside', action='store_true')
    parser.add_argument('--cancel-file')
    args = parser.parse_args()
    io = None
    try:
        cancel = cancel_path(args.cancel_file, args.inside) if args.action in ('on', 'park') else None
        if args.inside:
            io = InsideIO(); result = execute(args.action, io, lambda: cancelled_file(cancel))
        else:
            require(Path(__file__).resolve().parent.parent == HOST_ROOT, 'Run in the owned host project')
            result = host(args.action, cancel)
    except BaseException as error:
        result = dict(kind='archive_output_receipt_v1', action=args.action, success=False,
                      error=type(error).__name__+': '+str(error), hardware_commands=False)
    finally:
        if io is not None: io.close()
    print(('ARCHIVE_OUTPUT_RECEIPT=' if args.inside else '')+json.dumps(result, allow_nan=False), flush=True)
    return 0 if result.get('success') is True else 1


if __name__ == '__main__':
    raise SystemExit(main())
