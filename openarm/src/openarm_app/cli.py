import argparse
import json
import numpy as np
from openarm_app.kinematics import OpenArmIK, IKConvergenceError, pose_error
from openarm_app.trajectory import JointTrajectoryPlanner


def main():
    parser = argparse.ArgumentParser(description="Offline V1 arm planning (no motor communication)")
    parser.add_argument("--xml", help="Explicit V1 scene.xml path")
    parser.add_argument("--arm", choices=("right", "left"), default="right")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("inspect")
    fk = sub.add_parser("fk")
    fk.add_argument("--q", type=float, nargs=7, required=True)
    ik = sub.add_parser("ik")
    ik.add_argument("--pose", type=float, nargs=7, required=True)
    ik.add_argument("--seed", type=float, nargs=7, required=True)
    plan = sub.add_parser("plan")
    plan.add_argument("--start", type=float, nargs=7, required=True)
    goal = plan.add_mutually_exclusive_group(required=True)
    goal.add_argument("--goal", type=float, nargs=7)
    goal.add_argument("--pose", type=float, nargs=7)
    plan.add_argument("--velocity", type=float, nargs=7, required=True)
    plan.add_argument("--acceleration", type=float, nargs=7, required=True)
    plan.add_argument("--period", type=float, default=0.05)
    plan.add_argument("--csv", default="trajectory.csv")
    demo = sub.add_parser("demo")
    demo.add_argument("--csv", default="trajectory.csv")
    args = parser.parse_args()
    try:
        arm = OpenArmIK(args.xml, side=args.arm)
        if args.command == "inspect":
            result = dict(arm=args.arm, xml=str(arm.xml), home=arm.home.tolist(), limits=arm.limits.tolist(),
                qpos_indices=arm.setup.joint_resolver.arm_qpos_indices(args.arm).tolist(),
                pose=arm.forward(arm.home).tolist(), gripper=arm.gripper)
        elif args.command == "fk":
            result = arm.forward(args.q).tolist()
        elif args.command == "ik":
            result = arm.inverse(args.pose, args.seed).tolist()
        else:
            if args.command == "demo":
                start = arm.home
                target = arm.forward(start + np.array([0.1, 0.1, -0.1, 0.1, 0.05, 0.05, -0.05]))
                end = arm.inverse(target, start)
                velocity, acceleration, period = np.full(7, 0.2), np.full(7, 0.4), 0.05
            else:
                start = args.start
                end = args.goal if args.goal is not None else arm.inverse(args.pose, start)
                velocity, acceleration, period = args.velocity, args.acceleration, args.period
            trajectory = JointTrajectoryPlanner(arm.limits[:, 0], arm.limits[:, 1]).plan(
                start, end, velocity, acceleration, sample_period=period)
            trajectory.save_csv(args.csv)
            result = dict(csv=args.csv, duration_s=float(trajectory.time[-1]),
                          samples=len(trajectory.time), goal=list(end))
            if args.command == "demo":
                result["fk_error_m_rad"] = pose_error(arm.forward(end), target)
        print(json.dumps(result, indent=2))
    except (ValueError, RuntimeError, OSError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
