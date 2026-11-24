"""Both-arm zero/start-pose recipe for the local console.

Right-arm targets (left mirrors with J1/J2/J4/J6/J7 same sign and J3/J5
opposite; J3/J5 are zero here, so both arms use the same values):

1. zero pose: J1..J6 = 0, J7 = +15;
2. J1 +15 and J2 -45 together, 3 s;
3. J1 -30 (absolute), J2 stays -45, J4 +90, 3 s;
4. J1 stays -30, J2 back to 0, J4 +120, 3 s -- the start pose.

The quick buttons go directly to step 1 or step 4.
"""
from dataclasses import dataclass

ZERO_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0)
START_DEG = (-30.0, 0.0, 0.0, 120.0, 0.0, 0.0, 15.0)

ZERO_SECONDS = 3.0
J1_J2_SECONDS = 3.0
J1_J4_SECONDS = 3.0
START_SECONDS = 3.0


@dataclass(frozen=True)
class PoseStep:
    name: str
    right_deg: tuple
    left_deg: tuple
    duration_s: float


def return_start_steps():
    """The installation-constrained order from free state to the start pose."""
    return (
        PoseStep("0位（J1～J6 为 0°，J7 +15°）", ZERO_DEG, ZERO_DEG, ZERO_SECONDS),
        PoseStep("J1 +15° / J2 −45°", (15., -45., 0., 0., 0., 0., 15.),
                 (15., -45., 0., 0., 0., 0., 15.), J1_J2_SECONDS),
        PoseStep("J1 −30° / J4 +90°", (-30., -45., 0., 90., 0., 0., 15.),
                 (-30., -45., 0., 90., 0., 0., 15.), J1_J4_SECONDS),
        PoseStep("起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）",
                 START_DEG, START_DEG, START_SECONDS),
    )


def start_pose_step():
    """Shortcut: go straight to the calibrated start pose in both arms."""
    return PoseStep("起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）",
                    START_DEG, START_DEG, START_SECONDS)


def zero_step():
    """Shortcut: both arms straight to the revised mechanical zero."""
    return PoseStep("0位（J1～J6 为 0°，J7 +15°）", ZERO_DEG, ZERO_DEG, ZERO_SECONDS)
