"""反馈驱动的复位/收臂顺序。纯计算模块，不创建 ROS 或 CAN 写入源。"""
import math

SIDES = ('left', 'right')
FEEDBACK_BOUNDARY_TOLERANCE_RAD = math.radians(2)


def finite_vector(q):
    return (isinstance(q, (list, tuple)) and len(q) == 7
            and all(type(v) in (float, int) and math.isfinite(v) for v in q))


def validate_pose(q, limits):
    if not finite_vector(q) or any(not lo <= v <= hi for v, (lo, hi) in zip(q, limits)):
        raise ValueError('关节目标缺失、非有限值或超出模型限位')


def legal_feedback_seed(q, limits):
    """小幅实测越界只用于恢复起点；返回合法命令，不改写传入反馈。"""
    if not finite_vector(q):
        raise ValueError('关节反馈缺失或非有限值')
    result = []
    for i,(value,(lo,hi)) in enumerate(zip(q,limits)):
        legal = min(hi,max(lo,value))
        if abs(legal-value) > FEEDBACK_BOUNDARY_TOLERANCE_RAD + 1e-12:
            raise ValueError(f'J{i+1} 反馈越界 {math.degrees(abs(legal-value)):.2f}°，超过 2° 恢复范围')
        result.append(legal)
    validate_pose(result,limits)
    return result


def phases(start, home, config, parking=False):
    """J2 先向两侧抬臂，到位后其余关节复位；收臂时 J2 最后放下。"""
    result = []
    lifted = {s: [home[s][i] if i == 1 else q[i] for i in range(7)] for s,q in start.items()}
    result.extend([('reset_lift_j2', lifted),
                   ('reset_other_joints', {s: list(home[s]) for s in start})])
    if parking:
        result.extend([
            ('park_other_joints', {s: [home[s][i] if i == 1 else config['hanging'][s][i]
                                      for i in range(7)] for s in start}),
            ('park_lower_j2', {s: list(config['hanging'][s]) for s in start})])
    for _, targets in result:
        for s, q in targets.items():
            validate_pose(q, config['limits'][s])
    return result


class StagedMotion:
    """双臂收臂共享阶段屏障；计时结束不等于实际到位。"""
    def __init__(self, start, home, config, now, parking=False):
        self.config = config
        self.arrival_tolerance = list(config.get('arrival_tolerance_rad_by_joint',
                                                [config['arrival_tolerance_rad']]*7))
        if (not finite_vector(self.arrival_tolerance)
                or any(not 0 < v < config['tracking_error_rad'] for v in self.arrival_tolerance)):
            raise ValueError('逐关节到位容差必须有效且小于持续跟随超差阈值')
        self.settle_span = config.get('settle_motion_span_rad', math.radians(.5))
        if type(self.settle_span) not in (float,int) or not math.isfinite(self.settle_span) or self.settle_span <= 0:
            raise ValueError('反馈稳定范围必须为有限正数')
        self.feedback = None
        self.start_feedback = {s:list(q) for s,q in start.items()}
        self.command = {s:legal_feedback_seed(q,config['limits'][s]) for s,q in start.items()}
        self.start_boundary_returns = [{'side':s,'joint':i+1,'feedback_rad':a,'command_rad':b}
                                       for s in start for i,(a,b) in enumerate(zip(start[s],self.command[s])) if a != b]
        # 首个保持目标、J2 阶段及插值均从合法命令起点构造。
        # 已经越界的腕部仅收到向限位内回退的目标，其余顺序不变。
        self.steps = phases(self.command, home, config, parking)
        self.index = -1
        self.status = 'running'
        self.error = ''
        self.failure_code = None
        self.last_tick = now
        self.bad_since = None
        self._next(now)

    def _next(self, now):
        self.index += 1
        self.stable_since = None
        self.stable_low = self.stable_high = None
        self.bad_since = None
        if self.index == len(self.steps):
            self.status = 'complete'
            return
        self.phase, self.target = self.steps[self.index]
        self.start = {s: list(q) for s, q in self.command.items()}
        # 五次多项式解析速度/加速度峰值，避免大角度动作仍固定 3 秒。
        duration = self.config['min_duration_s']
        for s in self.start:
            for i, (a, b) in enumerate(zip(self.start[s], self.target[s])):
                d = abs(b-a)
                duration = max(duration, 1.875*d/self.config['velocity'][i],
                               math.sqrt(5.774*d/self.config['acceleration'][i]))
        self.duration = duration
        self.began = now

    def fail(self, reason, code='external_stop'):
        self.status = 'failed'
        self.error = reason
        self.failure_code = code

    def tick(self, now, feedback, ages):
        if self.status != 'running':
            return None
        if not 0 <= now-self.last_tick <= self.config['max_tick_gap_s']:
            self.fail('动作线程间隔异常，已停止推进', 'tick_gap'); return None
        self.last_tick = now
        for s in self.start:
            if (not finite_vector(feedback.get(s)) or not isinstance(ages.get(s), (float, int))
                    or not 0 <= ages[s] <= self.config['feedback_max_age_s']):
                self.fail('关节反馈失效，未继续收臂或关闭使能', 'feedback_invalid'); return None
        self.feedback = {s: list(feedback[s]) for s in self.start}
        for s in self.start:
            try: legal_feedback_seed(feedback[s], self.config['limits'][s])
            except ValueError as error:
                self.fail(s+' '+str(error), 'feedback_outside_model'); return None
        # 持续跟随超差中止阶段；不会越过阻挡继续完成下一阶段。
        lag = max(abs(a-b) for s in self.start for a, b in zip(self.command[s], feedback[s]))
        if lag > self.config['tracking_error_rad']:
            if self.bad_since is None: self.bad_since = now
            if now-self.bad_since >= self.config['tracking_error_s']:
                self.fail('收臂跟随超差，请检查阻挡', 'tracking_error'); return None
        else:
            self.bad_since = None
        elapsed = now-self.began
        t = min(1., max(0., elapsed/self.duration))
        e = t*t*t*(10+t*(-15+6*t))
        self.command = {s: [a+(b-a)*e for a,b in zip(self.start[s], self.target[s])] for s in self.start}
        if elapsed >= self.duration:
            arrived = all(abs(a-b) <= tol for s in self.start
                          for a,b,tol in zip(self.target[s],feedback[s],self.arrival_tolerance))
            if arrived and self.bad_since is None:
                # 每关节独立容差；在容差内穿过目标不算稳定到位。
                if self.stable_since is not None:
                    for s in self.start:
                        self.stable_low[s] = [min(a,b) for a,b in zip(self.stable_low[s],feedback[s])]
                        self.stable_high[s] = [max(a,b) for a,b in zip(self.stable_high[s],feedback[s])]
                    if any(hi-lo > self.settle_span for s in self.start
                           for lo,hi in zip(self.stable_low[s],self.stable_high[s])):
                        self.stable_since = None
                if self.stable_since is None:
                    self.stable_since = now
                    self.stable_low = {s:list(feedback[s]) for s in self.start}
                    self.stable_high = {s:list(feedback[s]) for s in self.start}
                if now-self.stable_since >= self.config['settle_s']:
                    self._next(now)
            else:
                self.stable_since = None
            if self.status == 'running' and now-self.began > self.duration+self.config['arrival_timeout_s']:
                waiting = self._waiting_joints()
                detail = '、'.join(f"{x['side']} J{x['joint']} 偏差 {x['error_deg']:.1f}°（容差 {x['tolerance_deg']:.1f}°）"
                                  for x in waiting)
                self.fail('等待关节到位超时，未关闭使能：'+(detail or '反馈尚未稳定'), 'arrival_timeout'); return None
        return {s: list(q) for s,q in self.command.items()}

    def _waiting_joints(self):
        if self.feedback is None: return []
        return [{'side':s,'joint':i+1,'error_deg':math.degrees(abs(a-b)),
                 'tolerance_deg':math.degrees(tol)}
                for s in self.start for i,(a,b,tol) in enumerate(zip(self.target[s],self.feedback[s],self.arrival_tolerance))
                if abs(a-b) > tol]

    def snapshot(self):
        return {'status': self.status, 'phase': self.phase, 'error': self.error,
                'failure_code': self.failure_code,
                'stage': min(self.index+1, len(self.steps)), 'stages': len(self.steps),
                'target': self.target, 'feedback': self.feedback,
                'start_feedback': self.start_feedback,
                'start_boundary_returns': self.start_boundary_returns,
                'feedback_boundary_tolerance_rad': FEEDBACK_BOUNDARY_TOLERANCE_RAD,
                'arrival_tolerance_rad_by_joint': self.arrival_tolerance,
                'settle_motion_span_rad': self.settle_span,
                'waiting_joints': self._waiting_joints()}
