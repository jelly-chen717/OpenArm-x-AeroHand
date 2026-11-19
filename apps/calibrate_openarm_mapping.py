"""Fit motor/model angle pairs in radians, using separate validation measurements."""
import argparse
import json
import math
from pathlib import Path

from openarm_joint_mapping import finite


def fit_mapping(points, validation, *, min_span, max_error):
    min_span, max_error = finite(min_span), finite(max_error)
    if min_span <= 0 or max_error <= 0:
        raise ValueError('Measurement thresholds must be positive')
    if len(points) < 3 or len(validation) < 2:
        raise ValueError('Need at least three fit and two independent validation poses')
    pairs = [(finite(p), finite(q)) for p, q in points]
    checks = [(finite(p), finite(q)) for p, q in validation]
    p_mean = sum(p for p, q in pairs) / len(pairs)
    q_mean = sum(q for p, q in pairs) / len(pairs)
    if max(p for p, q in pairs) - min(p for p, q in pairs) < min_span:
        raise ValueError('Motor measurement span too small')
    if max(p for p, q in checks) - min(p for p, q in checks) < min_span:
        raise ValueError('Validation span too small')
    denominator = sum((p - p_mean)**2 for p, q in pairs)
    slope = finite(sum((p-p_mean)*(q-q_mean) for p, q in pairs) / denominator)
    if abs(slope) < 1e-9:
        raise ValueError('Degenerate measured scale')
    offset = finite(q_mean - slope * p_mean)
    errors = [finite(q - (offset + slope*p)) for p, q in pairs]
    validation_errors = [finite(q - (offset + slope*p)) for p, q in checks]
    passed = max(abs(e) for e in errors + validation_errors) <= max_error
    return dict(mapping=dict(direction=1 if slope > 0 else -1, scale=abs(slope),
                             p_ref=p_mean, q_ref=q_mean),
                motor_angle_at_model_zero=finite(-offset/slope),
                fit_max_error_rad=max(abs(e) for e in errors),
                fit_rms_error_rad=math.sqrt(sum(e*e for e in errors)/len(errors)),
                validation_errors_rad=validation_errors, residual_check_passed=passed,
                hardware_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.measurements.read_text(encoding='utf-8'))
    if data.get('units') != 'rad' or not data.get('measurement_method'):
        raise ValueError('Specify rad units and independent angle measurement method')
    result = fit_mapping(data['fit_pairs'], data['validation_pairs'],
                         min_span=data['min_span_rad'], max_error=data['max_error_rad'])
    result.update(measurements=str(args.measurements.resolve()),
                  side=data['side'], motor_id=data['motor_id'],
                  measurement_method=data['measurement_method'])
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
