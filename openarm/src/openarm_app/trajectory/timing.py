"""Timing shared by the fixed-step planner, simulator and local controllers."""

INTERPOLATION_PERIOD_S = 1 / 60
DEFAULT_INTERPOLATION_STEPS = 300
MAX_INTERPOLATION_STEPS = 100_000


def interpolation_steps(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= MAX_INTERPOLATION_STEPS:
        raise ValueError(f"插补步数必须是 1–{MAX_INTERPOLATION_STEPS} 的整数")
    return value
