"""Compile console positioning and excerpts with the current physical piano calibration."""
from openarm_joint_mapping import finite
from openarm_piano_origin import SIDES, position_trial
from openarm_piano_performance import SCOPES, compile_performance


def compile_trial(payload, held_plan, mappings):
    scope = payload.get("scope")
    run = payload.get("anchor_run", 3)
    if type(run) is not int or run not in (1, 2, 3) or scope not in ("start", *SCOPES):
        raise ValueError("请选择要定位或试弹的范围。")
    if scope == "start":
        steps, info, fingers = position_trial(payload)
        return steps, info, fingers, None

    speed = finite(payload.get("speed", .35))
    if not .35 <= speed <= 1.:
        raise ValueError("试弹倍率范围为 0.35 至 1。")
    # Earlier C4 trial requests keep their right-hand scope on the current table.
    hand = payload.get("trial_hand", "right" if run in (1, 2) else "both")
    if hand not in ("both", "right"):
        raise ValueError("请选择双手或右手试弹。")
    return compile_performance(payload.get("score_id", "jin-chords"), held_plan, mappings,
                               scope=scope, speed=speed, sides=SIDES if hand == "both" else ("right",),
                               trial=True)
