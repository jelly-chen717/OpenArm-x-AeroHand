"""资料包原桥的复位/收臂适配；原 IK、遥操映射、夹爪与 CAN 驱动保持不变。

A/X：J2 先侧抬至原复位角，其余关节随后复位。网页正常关闭：先复位、
收直其余关节、J2 最后下放；每阶段按实际反馈到位。输出关闭立即取消动作。
原 jt_bridge.py 仍逐字节校验 SHA，所有目标沿用该节点的原发布者。
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import uuid
import math

from staged_arm_motion import StagedMotion, finite_vector, legal_feedback_seed


SOURCE_MODULE = 'openarm_teleop_bridge.jt_bridge'
SOURCE_SHA256 = '1e5d17c0dcaecd2ce46e1005d878d750978f9930180b1c348719f739b2612ccb'
STATE_SERVICE = '/archive_control/bridge_state'


# 注释：校验资料包桥源码 SHA 后执行刚校验的字节，避免复用旧 pyc。
def load_original():
    """Compile the checked installed source bytes, bypassing cached bytecode."""
    spec = importlib.util.find_spec(SOURCE_MODULE)
    if spec is None or not spec.origin:
        raise RuntimeError('Installed ZIP bridge source is unavailable')
    source = Path(spec.origin)
    if source.suffix != '.py' or not source.is_file():
        raise RuntimeError('Installed ZIP bridge must have a readable Python source')
    raw = source.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != SOURCE_SHA256:
        raise RuntimeError('Installed ZIP bridge source SHA256 differs')
    module = importlib.util.module_from_spec(spec)
    # Execute exactly the bytes just verified, not a pre-existing module/pyc.
    exec(compile(raw, str(source), 'exec'), module.__dict__)
    return module, {'source_module': SOURCE_MODULE, 'source_sha256': digest,
                    'source_origin': str(source), 'source_resolved_origin': str(source.resolve())}


# 注释：在原桥上创建窄适配子类，保留原关节、夹爪与复位映射。
def adapter_class(original, source_info):
    """Create the narrow subclass only after load_original verified the source."""
    if source_info.get('source_sha256') != SOURCE_SHA256:
        raise RuntimeError('ZIP bridge source proof is missing')
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import SetParametersAtomically

    # 注释：本模块的 adapter_class.ArchiveBridgeAdapter 定义；用途见文件说明与下方原实现。
    class ArchiveBridgeAdapter(original.JointTrajectoryBridge):
        # 注释：初始化 adapter_class.ArchiveBridgeAdapter 的状态与依赖；资源和服务创建以函数体为准。
        def __init__(self):
            super().__init__()
            # The original declares False. Reject a launch override before spin,
            # rather than start a bridge whose initial permission is already ON.
            if self._enabled is not False:
                self._enabled = False
                self.destroy_node()
                raise RuntimeError('Archive bridge requires enable_on_start=False')
            self._archive_source_info = dict(source_info)
            self._archive_instance_id = uuid.uuid4().hex
            self._archive_state_service = self.create_service(Trigger, STATE_SERVICE, self._archive_on_state)
            self._motion_config = json.loads(Path(__file__).with_name('staged_arm_motion.json').read_text(encoding='utf-8'))
            if self._motion_config.get('lift_joint') != 2:
                raise RuntimeError('分阶段复位必须使用 J2 侧抬配置')
            self._feedback_seen = {}
            self._reset_motion = {s: None for s in ('left', 'right')}
            self._reset_result = {}
            self._reset_recovery = {s: None for s in ('left', 'right')}
            self._park_motion = None
            self._park_owned = False
            self._park_id = None
            self._park_heartbeat = 0.
            self._park_cancelled_ids = []
            self._park_service = self.create_service(SetParametersAtomically, '/archive_control/park_command', self._park_command)

        def _park_command(self, request, response):
            """带事务 ID 和短有效期，迟到的开始请求不能在取消/OFF 后重新启动。"""
            try:
                if len(request.parameters) != 1:
                    raise ValueError('Invalid park parameters')
                p = request.parameters[0]
                if p.name != 'command' or p.value.type != 4:
                    raise ValueError('Invalid park payload')
                data = json.loads(p.value.string_value)
                identity, action, expires = data['id'], data['action'], data['expires']
                if (not isinstance(identity, str) or len(identity) != 32
                        or any(c not in '0123456789abcdef' for c in identity)):
                    raise ValueError('Invalid park identity')
                if action == 'cancel':
                    self._park_cancelled_ids = (self._park_cancelled_ids+[identity])[-128:]
                    if identity == self._park_id:
                        result = self._park_cancel(None, Trigger.Response())
                    else:
                        result = self._archive_on_state(None, Trigger.Response())
                else:
                    if (identity in self._park_cancelled_ids or type(expires) not in (float, int)
                            or not math.isfinite(expires) or not 0 <= expires-time.monotonic() <= 1.):
                        raise ValueError('收臂请求已取消或过期')
                    if action == 'start':
                        if identity == self._park_id: raise ValueError('收臂请求不能重放')
                        result = self._park_start(identity, Trigger.Response())
                    elif action == 'heartbeat' and identity == self._park_id:
                        result = self._park_pulse(None, Trigger.Response())
                    else: raise ValueError('收臂事务身份不符')
                response.result.successful = result.success
                response.result.reason = result.message
            except (ValueError, KeyError, TypeError) as error:
                response.result.successful = False
                response.result.reason = str(error)
            return response

        # 注释：关闭原桥时同时取消在途复位，再交给原实现处理许可。
        def _on_enable(self, request, response):
            if bool(request.data) and getattr(self, '_park_owned', False):
                response.success = False
                response.message = '收臂事务占用中，禁止重新开放遥操'
                return response
            if not bool(request.data):
                # The ZIP original only clears _enabled. Its timer otherwise
                # continues an already-started home, even after that False ACK.
                for side in ('left', 'right'):
                    self._home_start[side] = None
                    self._home_target[side] = None
                    self._home_begin[side] = None
                if hasattr(self, '_reset_motion'):
                    if self._park_id:
                        self._park_cancelled_ids = (self._park_cancelled_ids+[self._park_id])[-128:]
                    self._reset_motion = {s: None for s in ('left', 'right')}
                    self._reset_recovery = {s: None for s in ('left', 'right')}
                    if self._park_motion and self._park_motion.status == 'running':
                        self._park_motion.fail('输出许可已关闭')
                    self._park_owned = False
            return super()._on_enable(request, response)

        def _on_joint_states(self, msg):
            super()._on_joint_states(msg)
            now = time.monotonic()
            if not hasattr(self, '_feedback_seen'): return
            for name, value in zip(msg.name, msg.position):
                if math.isfinite(value): self._feedback_seen[name] = now
                else: self._feedback_seen.pop(name, None)

        def _feedback(self, sides):
            now = time.monotonic()
            q = {s: self._current_arm_q(s) for s in sides}
            ages = {s: max(now-self._feedback_seen.get(n, -1e9) for n in self._arm_joints[s]) for s in sides}
            if any(not finite_vector(q[s]) or not 0 <= ages[s] <= self._motion_config['feedback_max_age_s'] for s in sides):
                raise RuntimeError('需要新鲜的完整关节反馈')
            return q, ages

        def _mark_owned(self, sides, q):
            for s in sides:
                self._home_start[s] = list(q[s])
                self._home_target[s] = list(self._home[s])
                self._home_begin[s] = time.monotonic()

        def _publish_positions(self, q):
            from std_msgs.msg import Float64MultiArray
            for s, values in q.items(): self._pub_arm[s].publish(Float64MultiArray(data=list(values)))

        def _hold_feedback(self, sides):
            # 异常时撤销未跟上的运动目标，只有反馈新鲜才更新到当前位置保持。
            try:
                q = self._feedback(sides)[0]
                legal = {s:legal_feedback_seed(q[s], self._motion_config['limits'][s]) for s in sides}
                self._publish_positions(legal)
                return legal
            except (RuntimeError, ValueError): return None

        def _command_home(self, side):
            if not self._enabled or self._park_owned: return
            try:
                q, _ = self._feedback([side])
                motion = StagedMotion(q, self._home, self._motion_config, time.monotonic())
            except (RuntimeError, ValueError) as error:
                self.get_logger().warn('复位未启动: '+str(error)); return
            self._reset_motion[side] = motion
            self._reset_recovery[side] = None
            self._mark_owned([side], q)

        def _cancel_home(self, side):
            if self._park_owned: return
            recovery = self._reset_recovery[side]
            if recovery and not recovery['regripped']: return
            held = self._hold_feedback([side])
            if recovery and held is None: return
            self._reset_motion[side] = None
            self._reset_recovery[side] = None
            super()._cancel_home(side)

        def _on_trajectory(self, msg):
            if self._park_owned: return
            return super()._on_trajectory(msg)

        def _on_joy(self, side, msg):
            if self._park_owned:
                # 收臂期间屏蔽夹爪/复位/握持，维护边沿，避免结束后的旧按键触发。
                self._home_button_edge(side, msg)
                return
            recovery = self._reset_recovery[side]
            if self._enabled and recovery:
                recovery['regripped'] = False
                # 必须收到超时之后的新鲜“松手→重握”；缺失按键、旧帧不算松手。
                idx = int(self.get_parameter('home_cancel_button_index').value)
                stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                now_ns = self.get_clock().now().nanoseconds
                if (0 <= idx < len(msg.buttons) and stamp > recovery['after_ros_ns']
                        and 0 <= (now_ns-stamp)/1e9 <= self._motion_config['feedback_max_age_s']):
                    if not self._cancel_pressed(msg):
                        recovery['released_ros_ns'] = stamp
                    elif (recovery['released_ros_ns'] is not None
                          and stamp > recovery['released_ros_ns']):
                        recovery['regripped'] = True
            return super()._on_joy(side, msg)

        def _park_start(self, request, response):
            try:
                if self._enabled or self._park_owned:
                    raise RuntimeError('收臂开始前必须关闭遥操桥，且无在途收臂')
                q, _ = self._feedback(('left', 'right'))
                motion = StagedMotion(q, self._home, self._motion_config, time.monotonic(), parking=True)
                self._park_id = request if isinstance(request, str) else uuid.uuid4().hex
                self._park_motion = motion
                self._park_owned = True
                self._enabled = True  # 只授予已独占的收臂阶段；遥操与 Joy 仍被屏蔽。
                self._park_heartbeat = time.monotonic()
                self._reset_motion = {s: None for s in ('left', 'right')}
                self._reset_recovery = {s: None for s in ('left', 'right')}
                self._mark_owned(q, q)
                response.success = True
            except (ValueError, RuntimeError) as error:
                response.success = False
                response.message = str(error)
                return response
            response.message = json.dumps(self._archive_state(), allow_nan=False)
            return response

        def _park_pulse(self, request, response):
            self._park_heartbeat = time.monotonic()
            return self._archive_on_state(request, response)

        def _park_cancel(self, request, response):
            if self._park_owned:
                self._hold_feedback(('left', 'right'))
                if self._park_motion.status == 'running': self._park_motion.fail('收臂已取消')
                self._enabled = False
                for side in ('left', 'right'): super()._finish_home(side)
            return self._archive_on_state(request, response)

        # J2 阶段反馈到位且稳定后才推进。收臂运行时需内部事务持续健康检查。
        def _home_tick(self):
            if not self._enabled:
                return
            now = time.monotonic()
            if self._park_owned:
                motion = self._park_motion
                if now-self._park_heartbeat > self._motion_config['lease_timeout_s']:
                    motion.fail('收臂监督连接中断')
                if motion.status == 'running':
                    try:
                        q, ages = self._feedback(('left', 'right'))
                        command = motion.tick(now, q, ages)
                        if command: self._publish_positions(command)
                    except RuntimeError as error: motion.fail(str(error))
                if motion.status == 'failed':
                    self._hold_feedback(('left', 'right'))
                    self._enabled = False
                    for s in ('left', 'right'): super()._finish_home(s)
                # 完成仍保留占用直到显式 OFF，防止收臂后手柄重新写入。
                return
            for side, motion in list(self._reset_motion.items()):
                if motion is None: continue
                try:
                    q, ages = self._feedback([side])
                    command = motion.tick(now, q, ages)
                    if command: self._publish_positions(command)
                except RuntimeError as error: motion.fail(str(error))
                if motion.status != 'running':
                    self._reset_result[side] = motion.snapshot()
                    self._reset_motion[side] = None
                    if motion.status == 'failed':
                        held = self._hold_feedback([side])
                        if (motion.failure_code == 'arrival_timeout' and motion.bad_since is None
                                and held is not None):
                            # 普通复位的小残差超时只暂停该侧；不关闭共用桥或夹爪。
                            # 保留原桥的单臂占用，避免旧 IK 目标立即接管；重握才释放。
                            self._home_start[side] = list(held[side])
                            self._home_target[side] = list(held[side])
                            self._reset_recovery[side] = {
                                'after_ros_ns': self.get_clock().now().nanoseconds,
                                'released_ros_ns': None, 'regripped': False}
                            self.get_logger().warn(side+' 复位未完全到位，已保持；松手重握或按 A/X 重试')
                            continue
                        # 反馈失效、线程异常、持续跟随超差仍关闭输出，不自动恢复。
                        self._enabled = False
                        for s in ('left', 'right'):
                            self._reset_motion[s] = None
                            self._reset_recovery[s] = None
                            super()._finish_home(s)
                        self.get_logger().error('复位停止: '+motion.error)
                        break
                    super()._finish_home(side)

        # 注释：生成带实例标识、单调时间和复位目标的桥状态快照。
        def _archive_state(self):
            active = [side for side in ('left', 'right') if self._home_start[side] is not None]
            return {'kind': 'archive_control_bridge_state_v1', **self._archive_source_info,
                    'instance_id': self._archive_instance_id, 'heartbeat_monotonic_s': time.monotonic(),
                    'bridge_enabled': bool(self._enabled), 'home_active': bool(active),
                    'home_active_sides': active,
                    'home_targets': {side: None if self._home_target[side] is None else list(self._home_target[side])
                                     for side in ('left', 'right')},
                    'reset_sequence': 'j2_first_feedback_gated_v1',
                    'reset_recovery_version': 'arrival_timeout_per_arm_regrip_v1',
                    'reset_recovery': {s: {'state': 'regrip_required' if r['released_ros_ns'] is not None else 'release_required',
                                          'reason': 'arrival_timeout'}
                                       for s,r in self._reset_recovery.items() if r},
                    'reset_stages': {s: motion.snapshot() for s,motion in self._reset_motion.items() if motion},
                    'reset_last_result': self._reset_result,
                    'parking': None if self._park_motion is None else dict(self._park_motion.snapshot(),
                                id=self._park_id, owned=self._park_owned)}

        # 注释：通过只读 Trigger 服务返回桥状态，不改变输出许可。
        def _archive_on_state(self, request, response):
            response.success = True
            response.message = json.dumps(self._archive_state(), allow_nan=False, separators=(',', ':'))
            return response

    return ArchiveBridgeAdapter


# 注释：该文件的显式执行入口；逐字节校验原 jt_bridge.py；继承其关节、夹爪、A/X 复位，只补关闭时取消复位。
def main(args=None):
    original, source_info = load_original()
    bridge_type = adapter_class(original, source_info)
    import rclpy
    rclpy.init(args=args)
    node = None
    try:
        node = bridge_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
