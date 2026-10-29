"""08 — What a REJECTED move looks like, and how to handle it.

Every move is checked BEFORE it is sent: target inside the workspace box,
speed/accel under the profile ceilings, joints inside limits. A violation
raises ur5e.safety.SafetyViolation and the robot does not move — nothing is
silently clamped, so a wrong number is visible immediately.

    from ur5e.safety import SafetyViolation
    try:
        robot.move_to(pose)
    except SafetyViolation as e:
        print("rejected:", e)

What this does: an accepted move, then three deliberately bad ones
(outside the box, too fast, bad joint value) — each is rejected with a
readable message — then home.

Usage:  python examples/function_templates/08_safety_rejections.py
        python examples/function_templates/08_safety_rejections.py --target real   (lab, supervised)
"""

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.safety import SafetyViolation



def attempt(label, fn):
    try:
        fn()
        print(f"  [moved]    {label}")
    except SafetyViolation as e:
        print(f"  [rejected] {label}\n             -> {e}")


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        robot.move_joints(ws.home_joints)
        print("profile:", robot.limits.name, "| tcp speed max", robot.limits.tcp_speed_max, "m/s")

        attempt("centre of the box at 30 % height (fine)",
                lambda: robot.move_to(ws.pose(0.5, 0.5, 0.3)))

        outside = ws.pose(1.0, 1.0, 1.0)
        outside[0] += 0.30                                   # 30 cm beyond the box edge
        attempt("30 cm outside the box in x",
                lambda: robot.move_to(outside))

        attempt("centre again but at 2.0 m/s (over the ceiling)",
                lambda: robot.move_to(ws.pose(0.5, 0.5, 0.3), speed=2.0))

        bad_q = list(ws.home_joints)
        bad_q[1] += 10.0                                     # way past +-2*pi rad -> joint limit
        attempt("a joint target beyond its limit",
                lambda: robot.move_joints(bad_q))

        robot.move_joints(ws.home_joints)
    print("done — the robot only did the first move and went home.")


if __name__ == "__main__":
    main()
