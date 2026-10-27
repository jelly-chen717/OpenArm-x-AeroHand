"""Execute an absolute schedule, never burst commands to catch up."""


def execute(plan, cfg, hand, arm, clock, log=print):
    origin = clock.now()
    timing = cfg["timing"]
    primary = None
    cleanup_errors = []
    try:
        hand.connect()
        hand.read_pose()
        arm.connect()
        origin = clock.now()
        for action in plan["actions"]:
            deadline = origin + action["start"]
            clock.sleep(deadline - clock.now())
            if clock.now() - deadline > timing["late_tolerance_seconds"]:
                raise TimeoutError("演奏落后于计划，停止而不追赶节拍")
            kind, duration = action["kind"], action["duration"]
            info = str(action.get("pitch", action.get("group", "")))
            log(f"[{action['start']:8.3f}s] {kind:7} {info}")
            if kind in ("prepare", "release"):
                hand.release(duration)
            elif kind == "move":
                if not hand.released:
                    raise RuntimeError("未释放琴键，禁止机械臂移动")
                arm.move_to(action["group"], duration)
                arm.wait_until_arrived(action["group"], duration + timing["late_tolerance_seconds"])
            elif kind == "press":
                if arm.group != action["group"]:
                    raise RuntimeError("机械臂未确认到达目标位置，禁止按键")
                hand.press(action["pitch"], duration)
            else:
                raise ValueError(f"未知动作 {kind}")
            if clock.now() - (deadline + duration) > timing["late_tolerance_seconds"]:
                raise TimeoutError("动作执行超时，停止演奏")
        clock.sleep(origin + plan["total_seconds"] - clock.now())
    except (Exception, KeyboardInterrupt) as exc:
        primary = exc
    finally:
        # Stop an in-flight arm first; each cleanup attempt is independent.
        for name, fn in (("停止机械臂", arm.stop),
                         ("释放机械手", lambda: hand.release(timing["stop_seconds"]) if not hand.released else None),
                         ("关闭机械臂连接", arm.close), ("关闭机械手连接", hand.close)):
            try:
                fn()
            except (Exception, KeyboardInterrupt) as exc:
                cleanup_errors.append(f"{name}失败：{exc}")
        for msg in cleanup_errors:
            log(msg)
    if primary is not None:
        raise primary
    if cleanup_errors:
        raise RuntimeError("；".join(cleanup_errors))
