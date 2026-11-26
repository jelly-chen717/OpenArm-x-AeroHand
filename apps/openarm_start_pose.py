"""Both-arm zero/start-pose recipe for the local console.

Right-arm targets (left mirrors with J1/J2/J4/J6/J7 same sign and J3/J5
opposite; J3/J5 are zero here, so both arms use the same values):

1. zero pose: J1..J6 = 0, J7 = +15;
2. J1 +15 and J2 -45 together, 3 s;
3. J1 -30 (absolute), J2 stays -45, J4 +90, 3 s;
4. J1 stays -30, J2 back to 0, J4 +120, 3 s -- the start pose.

The quick buttons go directly to step 1 or step 4. ``return_zero_steps`` replays
this recipe backwards, so the arms leave the piano along the path they entered it.
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


def return_zero_steps():
    """Reverse of return_start_steps: start pose first, then the approach in reverse order.

    Every waypoint is the previous recipe's waypoint, and every joint delta is
    the negative of the forward delta, so the arms leave the piano along the
    exact path they entered it:

    1. current pose → start pose (J1 -30, J2 0, J4 +120);
    2. J1 -30 / J2 -45 / J4 +90 (J2 -45 and J4 -30 back from step 1);
    3. J1 +15 / J2 -45 (J1 +45 and J4 -90 back from step 2);
    4. 0位 (J1 -15 and J2 +45 back from step 3).

    The console holds the last target enabled, so the arms stop at 0位 live.
    """
    return (
        PoseStep("当前位置 → 起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）",
                 START_DEG, START_DEG, START_SECONDS),
        PoseStep("J1 −30° / J2 −45° / J4 +90°", (-30., -45., 0., 90., 0., 0., 15.),
                 (-30., -45., 0., 90., 0., 0., 15.), J1_J4_SECONDS),
        PoseStep("J1 +15° / J2 −45°", (15., -45., 0., 0., 0., 0., 15.),
                 (15., -45., 0., 0., 0., 0., 15.), J1_J2_SECONDS),
        PoseStep("0位（J1～J6 为 0°，J7 +15°）", ZERO_DEG, ZERO_DEG, ZERO_SECONDS),
    )


def start_pose_step():
    """Shortcut: go straight to the calibrated start pose in both arms."""
    return PoseStep("起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）",
                    START_DEG, START_DEG, START_SECONDS)


def zero_step():
    """Shortcut: both arms straight to the revised mechanical zero."""
    return PoseStep("0位（J1～J6 为 0°，J7 +15°）", ZERO_DEG, ZERO_DEG, ZERO_SECONDS)
