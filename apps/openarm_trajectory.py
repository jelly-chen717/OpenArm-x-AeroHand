"""Finite, time-parameterized model-space trajectories with reversible mapping."""
from dataclasses import dataclass
import math

from openarm_joint_mapping import finite


@dataclass(frozen=True)
class Trajectory:
    request_id: str
    keys: tuple
    points: tuple
    expires_at: float

    @property
    def duration(self):
        return self.points[-1][0]

    def sample(self, elapsed):
        elapsed = finite(elapsed)
        if elapsed < 0:
            raise ValueError('Negative trajectory time')
        for (t0, q0), (t1, q1) in zip(self.points, self.points[1:]):
            if elapsed < t1:
                u = max(0., (elapsed-t0)/(t1-t0))
                blend = 10*u**3 - 15*u**4 + 6*u**5
                rate = 30*u*u*(1-u)**2/(t1-t0)
                return (tuple(a+(b-a)*blend for a,b in zip(q0,q1)),
                        tuple((b-a)*rate for a,b in zip(q0,q1)))
        return self.points[-1][1], (0.,)*len(self.keys)


def preflight(request, version, mappings, current, now, seen):
    """Validate everything before the caller enables a transport."""
    request_id = request.get('request_id')
    if not isinstance(request_id, str) or not request_id.strip() or request_id in seen:
        raise ValueError('Missing or duplicate request ID')
    if request.get('mapping_version') != version:
        raise ValueError('Mapping version mismatch')
    expires = finite(request['expires_at'])
    if expires <= finite(now):
        raise ValueError('Expired request')
    if any(type(j['motor_id']) is not int for j in request['joints']):
        raise ValueError('Motor IDs must be integers')
    keys = tuple((j['side'], j['motor_id']) for j in request['joints'])
    if not keys or len(set(keys)) != len(keys) or any(k not in mappings for k in keys):
        raise ValueError('Invalid trajectory joints')
    limits = request['limits']
    vmax, amax, start_error = (finite(limits[k]) for k in
                             ('velocity_rad_s', 'acceleration_rad_s2', 'start_error_rad'))
    if min(vmax, amax, start_error) <= 0:
        raise ValueError('Limits must be positive')
    points = tuple((finite(p['t']), tuple(finite(q) for q in p['q'])) for p in request['points'])
    if len(points) < 2 or points[0][0] != 0 or points[-1][0] <= 0:
        raise ValueError('Need a trajectory starting at zero with positive duration')
    for t, q in points:
        if len(q) != len(keys):
            raise ValueError('Trajectory dimension mismatch')
        for key, value in zip(keys, q):
            if not mappings[key].limits[0] <= value <= mappings[key].limits[1]:
                raise ValueError('Model joint limit exceeded')
            mappings[key].to_motor(value)
    for (t0, q0), (t1, q1) in zip(points, points[1:]):
        dt = t1 - t0
        if dt <= 0:
            raise ValueError('Trajectory time must increase')
        for a, b in zip(q0, q1):
            # Exact extrema for the rest-to-rest quintic interpolant.
            if 1.875*abs(b-a)/dt > vmax or (10/math.sqrt(3))*abs(b-a)/dt**2 > amax:
                raise ValueError('Interpolated velocity or acceleration exceeded')
    for key, q in zip(keys, points[0][1]):
        if abs(finite(current[key])-q) > start_error:
            raise ValueError('Start pose mismatch')
    return Trajectory(request_id, keys, points, expires)
