"""02 — move_joints(): joint-space move (UR: moveJ).

    robot.move_joints(q, speed=None, accel=None)
        q      six joint angles in RADIANS
        speed  rad/s   (default: the profile's maximum)
        accel  rad/s^2

All joints move together and arrive together; the TCP path is NOT a straight
line. Use it to go to a known configuration (home) or to turn one joint.
The framework checks the target joints against the joint limits and the
resulting flange position against the workspace box before sending it.

What this does: go home, turn the last joint (wrist 3) +30° and back, and
go home again. Watch it in the viewer (localhost:7001).

Usage:  python examples/function_templates/02_move_joints.py
        python examples/function_templates/02_move_joints.py --target real   (lab, supervised)
"""

import math

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

DELTA_DEG = 30.0


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("going home (taught joints) ...")
        robot.move_joints(ws.home_joints)

        q = list(robot.joints())
        q[5] += math.radians(DELTA_DEG)                # wrist 3 only
        print(f"wrist 3 +{DELTA_DEG:.0f} deg, slowly ...")
        robot.move_joints(q, speed=0.3, accel=0.5)     # explicit, gentle values

        print("and back at the default speed ...")
        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()
