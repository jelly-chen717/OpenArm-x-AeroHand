"""Dedicated trajectory executor with explicit virtual and PCAN modes."""
import argparse
from contextlib import ExitStack
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import threading
import time

from openarm_joint_mapping import finite, load_mapping
from openarm_trajectory import preflight

ROOT = Path(__file__).resolve().parents[1]


class VirtualTransport:
    def __init__(self, mappings, initial):
        self.mappings = mappings
        self.position = {key: mappings[key].to_motor(q) for key, q in initial.items()}
        self.velocity = dict.fromkeys(initial, 0.)
        self.enabled = False

    def read(self, keys):
        return {key: self.mappings[key].to_model(self.position[key]) for key in keys}

    def enable(self):
        self.enabled = True

    def command(self, targets):
        if not self.enabled:
            raise RuntimeError('Transport disabled')
        self.position.update({key: p for key, (p, v) in targets.items()})
        self.velocity.update({key: v for key, (p, v) in targets.items()})

    def read_velocity(self, keys):
        return {key: self.mappings[key].velocity_to_model(self.velocity[key]) for key in keys}

    def stop(self):
        self.enabled = False
        return True


def _checkpoints(trajectory, tolerance):
    if tolerance is None:
        return []
    return [(t, q) for index, (t, q) in enumerate(trajectory.points[1:], start=1)
            if index + 1 == len(trajectory.points) or trajectory.points[index + 1][1] != q]


def execute(trajectory, mappings, transport, log, *, period=.02, max_lateness=.1,
            max_error=None, cancel=None, seen=None, cancel_file=None, endpoint_tolerance=None,
            hold_after=False, next_trajectory=None, hold_requested=None,
            piano_clock=None, configure_cycle=None):
    """Send the trajectory until its nominal end, then optionally hold the target.

    A hold request replaces the active path with a fixed, measured pose.
    Thresholds (tracking error, waypoint/endpoint tolerance, cycle budget, expiry)
    are recorded as `advisory` rows. They no longer raise, so a lagging axis keeps
    receiving commands and the session stays enabled. Cancellation, transport
    errors and the stop/disable path are unchanged.
    """
    period, max_lateness = map(finite, (period, max_lateness))
    if min(period, max_lateness) <= 0:
        raise ValueError('Execution thresholds must be positive')
    cancel = cancel if cancel is not None else threading.Event()
    seen = seen if seen is not None else set()
    if trajectory.request_id in seen:
        raise ValueError('Duplicate execution')
    seen.add(trajectory.request_id)
    state = 'FAULT'
    checkpoints = _checkpoints(trajectory, endpoint_tolerance)
    checkpoint_index = 0
    holding_target = False
    hold_q = None
    advised = {}

    def advisory(kind, message, **data):
        now = time.perf_counter()
        if now - advised.get(kind, -1e9) < 1.:
            return
        advised[kind] = now
        log(dict(type='advisory', kind=kind, message=message, **data))

    try:
        if time.time() >= trajectory.expires_at:
            raise ValueError('Request expired before enable')
        if cancel.is_set() or (cancel_file is not None and cancel_file.exists()):
            state = 'CANCELLED'
            return state
        log(dict(type='state', state='HOLDING'))
        transport.enable()
        if cancel.is_set():
            state = 'CANCELLED'
            return state
        began = time.perf_counter()
        deadline = began
        clock = piano_clock() if piano_clock is not None else None
        if clock is not None:
            clock.start(began)
        log(dict(type='state', state='EXECUTING', started_at=began))
        while True:
            if cancel_file is not None and cancel_file.exists():
                state = 'CANCELLED'
                break
            if cancel.wait(max(0., deadline-time.perf_counter())):
                state = 'CANCELLED'
                break
            now = time.perf_counter()
            if hold_q is None and hold_requested is not None and hold_requested():
                current = transport.read(trajectory.keys)
                hold_q = tuple(current[key] for key in trajectory.keys)
                checkpoints = []
                holding_target = False
                clock = None
            if now-deadline > max_lateness:
                advisory('deadline', f'控制周期延迟 {now-deadline:.3f} s')
            wall_elapsed = now-began
            elapsed, cue = clock.sample(now) if clock is not None else (wall_elapsed, None)
            if hold_q is None and not holding_target and time.time() - (wall_elapsed - elapsed) >= trajectory.expires_at:
                advisory('expired', '请求有效期已过，继续执行当前目标')
            q, velocity = (hold_q, (0.,)*len(hold_q)) if hold_q is not None else trajectory.sample(elapsed)
            if cue is not None:
                velocity = (0.,) * len(q)
            if configure_cycle is not None:
                configure_cycle(trajectory if hold_q is None else None, elapsed)
            targets = {key: (mappings[key].to_motor(pos), mappings[key].velocity_to_motor(v))
                       for key, pos, v in zip(trajectory.keys, q, velocity)}
            transport.command(targets)
            measured = transport.read(trajectory.keys)
            if time.perf_counter()-deadline > period+max_lateness:
                advisory('cycle', '本周期的收发耗时超出节拍预算')
            error = [finite(measured[k])-target for k, target in zip(trajectory.keys, q)]
            sample_row = dict(type='sample', t=elapsed, target=q, target_velocity=velocity,
                              measured=[measured[k] for k in trajectory.keys], error=error)
            if clock is not None:
                sample_row['wall_elapsed_s'] = wall_elapsed
                if cue is not None:
                    read_at = time.perf_counter()
                    sync, changed = clock.observe(cue, measured, read_at)
                    if sync is not None:
                        sample_row['piano_sync'] = sync
                        if changed:
                            log(dict(type='piano_sync', wall_elapsed_s=read_at-began, **sync))
            if getattr(transport, 'last_dispatch', None) is not None:
                sample_row['dispatch'] = transport.last_dispatch
            log(sample_row)
            peak = max(abs(e) for e in error)
            if max_error is not None and peak > max_error:
                advisory('tracking_error', f'跟踪偏差 {peak:.4f} rad 超过 {max_error}', error=peak)
            while checkpoint_index < len(checkpoints) and elapsed >= checkpoints[checkpoint_index][0]:
                checkpoint_time,checkpoint_q = checkpoints[checkpoint_index]
                checkpoint_error = [finite(measured[k])-target for k,target in zip(trajectory.keys,checkpoint_q)]
                log(dict(type='checkpoint',t=elapsed,planned_t=checkpoint_time,
                         target=checkpoint_q,error=checkpoint_error))
                worst = max(abs(e) for e in checkpoint_error)
                if endpoint_tolerance is not None and worst > endpoint_tolerance:
                    advisory('waypoint', f'路径点偏差 {worst:.4f} rad 超过 {endpoint_tolerance}')
                checkpoint_index += 1
            if hold_q is not None or elapsed >= trajectory.duration:
                residual = max(abs(e) for e in error)
                if hold_q is None and endpoint_tolerance is not None and residual > endpoint_tolerance:
                    advisory('endpoint_gap', f'轨迹结束残差 {residual:.4f} rad 超过 {endpoint_tolerance}，'
                                             f'继续保持在目标角', error=residual)
                if not hold_after and hold_q is None:
                    state = 'COMPLETED'
                    break
                if not holding_target:
                    holding_target = True
                    log(dict(type='state', state='HOLDING_TARGET',
                             outcome='INTERRUPTED' if hold_q is not None else 'COMPLETED',
                             t=elapsed, target=q, measured=[measured[k] for k in trajectory.keys],
                             residual=residual))
                if next_trajectory is not None:
                    upcoming = next_trajectory()
                    if upcoming is not None:
                        if upcoming.request_id in seen or time.time() >= upcoming.expires_at:
                            raise ValueError('Duplicate or expired continuation')
                        seen.add(upcoming.request_id)
                        trajectory = upcoming
                        checkpoints = _checkpoints(trajectory, endpoint_tolerance)
                        checkpoint_index = 0
                        holding_target = False
                        hold_q = None
                        began = deadline = time.perf_counter()
                        clock = piano_clock() if piano_clock is not None else None
                        if clock is not None:
                            clock.start(began)
                        log(dict(type='state', state='EXECUTING', started_at=began))
                        continue
            # Skip elapsed deadlines. Never burst-send old commands.
            deadline = began + (int((time.perf_counter()-began)/period)+1)*period
    except BaseException as exc:
        state = 'FAULT'
        log(dict(type='fault', error=str(exc)))
        raise
    finally:
        try:
            log(dict(type='state', state='STOP_REQUESTED', outcome=state))
        finally:
            confirmed = transport.stop()
        log(dict(type='state', state='STOPPED' if confirmed else 'STOP_UNCONFIRMED', outcome=state))
        if not confirmed:
            raise RuntimeError('Stop unconfirmed')
    return state


def validate_sequence(trajectory, sequence):
    """Validate event-gated stages without accessing or enabling a transport."""
    if not isinstance(sequence, dict) or sequence.get('schema_version') != 1:
        raise ValueError('Sequence schema_version must be 1')
    dwell, timeout, speed = (finite(sequence[name]) for name in
                            ('dwell_s', 'settle_timeout_s', 'settle_speed_rad_s'))
    if min(dwell, timeout, speed) <= 0 or timeout <= dwell:
        raise ValueError('Sequence requires positive limits and timeout greater than dwell')
    tolerances = tuple(finite(v) for v in sequence['endpoint_tolerances_rad'])
    if len(tolerances) != len(trajectory.keys) or min(tolerances) <= 0:
        raise ValueError('Sequence tolerance dimension or value invalid')
    stages = sequence['stages']
    if not isinstance(stages, list) or len(stages) != len(trajectory.points)-1:
        raise ValueError('Sequence requires one stage per trajectory segment')
    names = set()
    for index, stage in enumerate(stages, 1):
        if (type(stage.get('endpoint_index')) is not int or stage['endpoint_index'] != index
                or not isinstance(stage.get('name'), str) or not stage['name'].strip()
                or stage['name'] in names):
            raise ValueError('Sequence stage names and ordered endpoints invalid')
        names.add(stage['name'])
        stage_tolerances = tuple(finite(v) for v in stage.get('endpoint_tolerances_rad', tolerances))
        if len(stage_tolerances) != len(trajectory.keys) or min(stage_tolerances) <= 0:
            raise ValueError('Stage tolerance dimension or value invalid')
        a, b = trajectory.points[index-1][1], trajectory.points[index][1]
        moving = [i for i, (start, end) in enumerate(zip(a, b)) if start != end]
        if not moving or any(stage_tolerances[i] >= abs(b[i]-a[i]) for i in moving):
            raise ValueError('Arrival tolerance must be smaller than each moving joint displacement')
    return dwell, timeout, speed, tolerances


def _measured_velocities(transport, mappings, keys):
    if hasattr(transport, 'read_velocity'):
        result = transport.read_velocity(keys)
    else:
        result = {key: mappings[key].velocity_to_model(transport.states[key].velocity_rad_s)
                  for key in keys}
    if set(result) != set(keys):
        raise RuntimeError('Measured velocity joints differ from trajectory')
    return tuple(finite(result[key]) for key in keys)


def execute_sequence(trajectory, mappings, transport, log, *, sequence, period=.02,
                     max_lateness=.1, max_error=.05, cancel=None, seen=None,
                     cancel_file=None, hold_after=True):
    """Move one segment, then wait for measured settling before the next.

    Every joint remains commanded in every cycle, including not-yet-moved joints.
    Arrival is still taken from measured feedback rather than nominal time, but a
    settlement timeout now advances with a logged `stage_timeout` instead of
    aborting; holding and tracking deviations are logged, not raised.
    """
    dwell, settle_timeout, settle_speed, tolerances = validate_sequence(trajectory, sequence)
    period, max_lateness, max_error = map(finite, (period, max_lateness, max_error))
    if min(period, max_lateness, max_error) <= 0:
        raise ValueError('Execution thresholds must be positive')
    cancel = cancel if cancel is not None else threading.Event()
    seen = seen if seen is not None else set()
    if trajectory.request_id in seen:
        raise ValueError('Duplicate execution')
    seen.add(trajectory.request_id)
    state = 'FAULT'
    stage_index = 0
    holding_target = False
    settled_since = None
    motion_ended = None
    advised = {}

    def advisory(kind, message, **data):
        now = time.perf_counter()
        if now - advised.get(kind, -1e9) < 1.:
            return
        advised[kind] = now
        log(dict(type='advisory', kind=kind, message=message, **data))

    try:
        if time.time() >= trajectory.expires_at:
            raise ValueError('Request expired before enable')
        if cancel.is_set() or (cancel_file is not None and cancel_file.exists()):
            state = 'CANCELLED'
            return state
        log(dict(type='state', state='HOLDING'))
        transport.enable()
        began = stage_began = deadline = time.perf_counter()
        log(dict(type='state', state='EXECUTING_SEQUENCE'))
        log(dict(type='stage_start', stage=sequence['stages'][0]['name'], index=0, t=0.))
        while True:
            if cancel_file is not None and cancel_file.exists():
                state = 'CANCELLED'
                break
            if cancel.wait(max(0., deadline-time.perf_counter())):
                state = 'CANCELLED'
                break
            now = time.perf_counter()
            if now-deadline > max_lateness:
                advisory('deadline', f'控制周期延迟 {now-deadline:.3f} s')
            if not holding_target and time.time() >= trajectory.expires_at:
                advisory('expired', '请求有效期已过，继续执行当前段')
            stage = sequence['stages'][stage_index]
            stage_tolerances = tuple(stage.get('endpoint_tolerances_rad', tolerances))
            t0, stage_start_q = trajectory.points[stage_index]
            t1, endpoint = trajectory.points[stage_index+1]
            move_duration = t1-t0
            stage_elapsed = now-stage_began
            moving = not holding_target and stage_elapsed < move_duration
            if moving:
                q, velocity = trajectory.sample(t0+stage_elapsed)
            else:
                q, velocity = endpoint, (0.,)*len(trajectory.keys)
            targets = {key: (mappings[key].to_motor(pos), mappings[key].velocity_to_motor(v))
                       for key, pos, v in zip(trajectory.keys, q, velocity)}
            transport.command(targets)
            measured = transport.read(trajectory.keys)
            measured_velocity = _measured_velocities(transport, mappings, trajectory.keys)
            observed = time.perf_counter()
            if observed-deadline > period+max_lateness:
                advisory('cycle', '本周期的收发耗时超出节拍预算')
            error = tuple(finite(measured[k])-target for k, target in zip(trajectory.keys, q))
            sample_row = dict(type='sample', t=observed-began, stage=stage['name'],
                              phase='holding_target' if holding_target else 'moving' if moving else 'settling',
                              target=q, target_velocity=velocity,
                              measured=[measured[k] for k in trajectory.keys],
                              measured_velocity=measured_velocity, error=error)
            if getattr(transport, 'last_dispatch', None) is not None:
                sample_row['dispatch'] = transport.last_dispatch
            log(sample_row)
            if max(abs(e) for e in error) > max_error:
                advisory('tracking_error', f'跟踪偏差 {max(abs(e) for e in error):.4f} rad 超过 {max_error}')
            # Only the active pair needs a convergence allowance. Already moved
            # and not-yet-moved axes must keep their targets throughout the motion.
            hold_errors = [dict(side=key[0], motor_id=key[1], error_rad=e, tolerance_rad=tol)
                           for key, start, end, e, tol in
                           zip(trajectory.keys, stage_start_q, endpoint, error, tolerances)
                           if (holding_target or start == end) and abs(e) > tol]
            if hold_errors:
                log(dict(type='hold_tolerance_exceeded', t=observed-began,
                         stage=stage['name'], joints=hold_errors))
            if not moving and not holding_target:
                if motion_ended is None:
                    motion_ended = observed
                    log(dict(type='stage_motion_complete', stage=stage['name'], t=observed-began,
                             planned_move_s=move_duration, actual_move_s=observed-stage_began))
                in_position = all(abs(e) <= tolerance for e, tolerance in zip(error, stage_tolerances))
                at_rest = all(abs(v) <= settle_speed for v in measured_velocity)
                if in_position and at_rest:
                    if settled_since is None:
                        settled_since = observed
                        log(dict(type='stage_arrived', stage=stage['name'], t=observed-began,
                                 wait_after_motion_s=observed-motion_ended, error=error,
                                 measured_velocity=measured_velocity))
                else:
                    if settled_since is not None:
                        log(dict(type='stage_dwell_reset', stage=stage['name'], t=observed-began,
                                 interrupted_dwell_s=observed-settled_since,
                                 error=error, measured_velocity=measured_velocity))
                    settled_since = None
                dwell_elapsed = 0. if settled_since is None else observed-settled_since
                overdue = observed-motion_ended > settle_timeout
                if overdue:
                    log(dict(type='stage_timeout', stage=stage['name'], t=observed-began,
                             wait_after_motion_s=observed-motion_ended, dwell_s=dwell_elapsed,
                             error=error, measured_velocity=measured_velocity))
                if (settled_since is not None and dwell_elapsed >= dwell) or overdue:
                    log(dict(type='stage_complete', stage=stage['name'], index=stage_index,
                             t=observed-began, planned_move_s=move_duration,
                             actual_move_s=motion_ended-stage_began,
                             arrival_wait_s=None if settled_since is None else settled_since-motion_ended,
                             dwell_s=dwell_elapsed, total_stage_s=observed-stage_began,
                             error=error, measured_velocity=measured_velocity, forced=overdue))
                    if stage_index+1 == len(sequence['stages']):
                        if not hold_after:
                            state = 'COMPLETED'
                            break
                        holding_target = True
                        log(dict(type='state', state='HOLDING_TARGET', outcome='COMPLETED',
                                 t=observed-began, target=q,
                                 measured=[measured[k] for k in trajectory.keys]))
                    else:
                        stage_index += 1
                        stage_began = observed
                        settled_since = motion_ended = None
                        log(dict(type='stage_start', stage=sequence['stages'][stage_index]['name'],
                                 index=stage_index, t=observed-began))
            deadline = began + (int((time.perf_counter()-began)/period)+1)*period
    except BaseException as exc:
        state = 'FAULT'
        log(dict(type='fault', error=str(exc)))
        raise
    finally:
        try:
            log(dict(type='state', state='STOP_REQUESTED', outcome=state))
        finally:
            confirmed = transport.stop()
        log(dict(type='state', state='STOPPED' if confirmed else 'STOP_UNCONFIRMED', outcome=state))
        if not confirmed:
            raise RuntimeError('Stop unconfirmed')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--mapping', type=Path, default=ROOT/'configs/openarm_joint_mapping.json')
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--period', type=float, default=.02)
    parser.add_argument('--kp', type=float, default=10.)
    parser.add_argument('--kd', type=float, default=2.)
    parser.add_argument('--joint-settings', type=Path,
                        help='JSON list of per-joint gains and protection thresholds')
    parser.add_argument('--limit-profile', choices=('rated', 'custom'), default='rated',
                        help='Hardware speed/torque limits: rated (24 V, overrides old settings) '
                             'or custom (use explicit task thresholds)')
    parser.add_argument('--endpoint-tolerance', type=float, default=.015,
                        help='Logged waypoint/endpoint deviation in radians (advisory)')
    parser.add_argument('--max-error', type=float, default=.05,
                        help='Logged tracking error in radians (advisory)')
    parser.add_argument('--max-lateness', type=float, default=.1,
                        help='Logged control deadline lateness in seconds (advisory)')
    parser.add_argument('--hold-after', action='store_true',
                        help='Keep sending the final target until cancelled; faults still disable')
    parser.add_argument('--allow-dual-arm', action='store_true',
                        help='Explicitly allow both arms under the same transport and trajectory clock')
    parser.add_argument('--batch-mit', action='store_true',
                        help='Send the entire validated MIT batch before collecting its feedback')
    parser.add_argument('--sequence', action='store_true',
                        help='Use request.sequence measured-arrival gates between movement segments')
    parser.add_argument('--mode', choices=('virtual','hardware'), default='virtual')
    parser.add_argument('--action', choices=('preflight','execute'), default='preflight')
    parser.add_argument('--left-channel', type=lambda s:int(s,0))
    parser.add_argument('--right-channel', type=lambda s:int(s,0))
    parser.add_argument('--confirm-bus-ownership', action='store_true')
    parser.add_argument('--cancel-file', type=Path)
    parser.add_argument('--mirror', action='store_true', help='Update MuJoCo from measured feedback headlessly')
    args = parser.parse_args()
    if args.hold_after and args.cancel_file is None:
        parser.error('--hold-after requires --cancel-file')
    if args.sequence and not args.hold_after:
        parser.error('--sequence requires --hold-after and --cancel-file')
    request = json.loads(args.trajectory.read_text(encoding='utf-8'))
    if 'sequence' in request and not args.sequence:
        parser.error('Request contains sequence metadata; --sequence is required')
    hardware = args.mode == 'hardware'
    if hardware and request.get('offline_only') is True:
        parser.error('Offline preparation candidate; rebuild after frozen-zero/model review and approval')
    if hardware and (not args.confirm_bus_ownership or args.left_channel is None or args.right_channel is None):
        parser.error('Hardware requires explicit channels and --confirm-bus-ownership')
    version, mappings = load_mapping(args.mapping,hardware=hardware)
    if len(request['initial_model_position']) != len(request['joints']):
        raise ValueError('Initial position dimension mismatch')
    initial = {(j['side'], j['motor_id']): finite(q) for j, q in
               zip(request['joints'], request['initial_model_position'])}
    transport = VirtualTransport(mappings, initial)
    if hardware:
        from openarm_execution import PcanTransport
        keys = tuple((j['side'], j['motor_id']) for j in request['joints'])
        if not keys or (len({key[0] for key in keys}) != 1 and not args.allow_dual_arm):
            parser.error('Hardware execution requires one arm or explicit --allow-dual-arm')
        settings = None
        if args.joint_settings is not None:
            rows = json.loads(args.joint_settings.read_text(encoding='utf-8'))
            settings = {}
            for row in rows:
                key = row['side'], row['motor_id']
                if type(key[1]) is not int or key in settings:
                    parser.error('Invalid or duplicate joint settings')
                settings[key] = {name: value for name, value in row.items()
                                 if name not in ('side', 'motor_id')}
        if (len(keys) != 1 or keys[0][1] != 7) and settings is None:
            parser.error('Expanded hardware execution requires --joint-settings')
        transport = PcanTransport(mappings,keys,dict(left=args.left_channel,right=args.right_channel),
                                  kp=args.kp,kd=args.kd,joint_settings=settings,
                                  limit_profile=args.limit_profile, batch_mit=args.batch_mit)
    mirror = None
    if args.mirror:
        from mirror_openarm_mujoco import Mirror
        mirror = Mirror(mappings)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        stream = stack.enter_context(args.log.open('x', encoding='utf-8'))
        if hardware:
            stack.enter_context(transport)
            initial = transport.read(keys)
            if mirror is not None:
                from openarm_feedback import Sample
                for arm in transport.before:
                    report = dict(side=arm['side'],valid=True,samples=[
                        Sample(m['motor_id'],m['position_rad'],m['velocity_rad_s'],m['received_at'],m['state'])
                        for m in arm['motors']])
                    if not mirror.apply(report):
                        raise RuntimeError('Initial measured pose rejected by MuJoCo')
        trajectory = preflight(request, version, mappings, initial, time.time(), set())
        if args.sequence:
            validate_sequence(trajectory, request.get('sequence'))
        if hardware:
            transport.validate_trajectory(trajectory)
        def log(row):
            if mirror is not None and row['type'] == 'sample':
                from openarm_feedback import Sample
                for side in {key[0] for key in trajectory.keys}:
                    samples = []
                    if hardware:
                        for key in trajectory.keys:
                            if key[0] == side:
                                s = transport.states[key]
                                samples.append(Sample(key[1],s.position_rad,s.velocity_rad_s,s.received_at,s.state))
                    else:
                        for motor_id in range(1, 8):
                            key = side, motor_id
                            # Other joints retain their virtual initial pose.
                            mapping = mappings[key]
                            qi, vi = mirror.indices[key]
                            position = transport.position.get(key, mapping.to_motor(float(mirror.data.qpos[qi])))
                            samples.append(Sample(motor_id, position, 0., time.monotonic()))
                    report = dict(side=side, valid=True, samples=samples)
                    if not mirror.apply(report):
                        raise RuntimeError('MuJoCo rejected feedback: '+str(report['reason']))
                row['mujoco_updated'] = True
                row['mujoco_live_joints'] = trajectory.keys
            if hardware and row['type'] == 'sample':
                row['motor_feedback'] = [dict(side=key[0],**asdict(transport.states[key])) for key in trajectory.keys]
            row.update(feedback_kind='hardware' if hardware else 'synthetic_virtual', request_id=trajectory.request_id)
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
        log(dict(type='session', mapping_version=version, keys=trajectory.keys,
                 source=request.get('source'),request=str(args.trajectory.resolve())))
        if args.sequence:
            log(dict(type='sequence_settings', **request['sequence']))
        if hardware:
            log(dict(type='preflight',before=transport.before,action=args.action,
                     kp=transport.kp,kd=transport.kd,max_travel_rad=transport.max_travel,
                     limit_profile=transport.limit_profile,
                     effective_limits=[dict(side=k[0], motor_id=k[1],
                                            max_speed_rad_s=v['max_speed'], max_torque_nm=v['max_torque'])
                                       for k,v in transport.joint_settings.items()],
                     max_temperature_c=transport.max_temperature,max_age_s=transport.max_age))
            log(dict(type='joint_settings', joints=[dict(side=key[0],motor_id=key[1],**values)
                                                    for key,values in transport.joint_settings.items()],
                     limit_profile=transport.limit_profile,
                     settings_source=str(args.joint_settings) if args.joint_settings else None,
                     period_s=args.period, endpoint_tolerance_rad=args.endpoint_tolerance,
                     max_tracking_error_rad=args.max_error,
                     max_lateness_s=args.max_lateness,
                     hold_after=args.hold_after,
                     command_order=keys,
                     dispatch='batch_send_then_collect' if args.batch_mit else 'sequential',
                     timing_basis='shared_trajectory_time_host_dispatch_not_device_simultaneity'))
            if args.action == 'preflight':
                return
            ledger = ROOT/'logs/openarm/executed_requests'
            ledger.mkdir(parents=True,exist_ok=True)
            claim = ledger/(hashlib.sha256(trajectory.request_id.encode()).hexdigest()+'.json')
            with claim.open('x',encoding='utf-8') as claimed:
                json.dump(dict(request_id=trajectory.request_id,log=str(args.log.resolve()),
                               time_unix=time.time()),claimed)
        try:
            if args.sequence:
                if args.action == 'preflight':
                    return
                execute_sequence(trajectory, mappings, transport, log, sequence=request['sequence'],
                                 period=args.period, max_error=args.max_error,
                                 max_lateness=args.max_lateness, hold_after=args.hold_after,
                                 cancel_file=args.cancel_file)
            else:
                execute(trajectory, mappings, transport, log, period=args.period,
                        max_error=args.max_error,
                        max_lateness=args.max_lateness,
                        hold_after=args.hold_after,
                        cancel_file=args.cancel_file, endpoint_tolerance=args.endpoint_tolerance if hardware else None)
        finally:
            if hardware:
                log(dict(type='disable_evidence',records=transport.stop_records))
                log(dict(type='after',reports=transport.require_disabled()))


if __name__ == '__main__':
    main()
