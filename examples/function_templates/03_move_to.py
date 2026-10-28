"""03 — move_to(): straight-line move of the TCP (UR: moveL).

    robot.move_to(pose, speed=None, accel=None)
        pose   [x, y, z, rx, ry, rz]  metres + rotation vector (rad), Base frame
        speed  m/s    (default: the profile's maximum — 0.25 in the sim)
        accel  m/s^2

The TCP travels on a straight line; the joints do whatever that needs.
Poses come from the workspace in fractions:  ws.pose(u, v, w)
    u = 0..1 along x of the box, v along y, w along z (0 = surface height)
The orientation is the one you taught (tool pointing down).

What this does: home → hover over the four corners at 30 % height → centre →
down to 5 % above the surface → up → home.

Usage:  python examples/function_templates/03_move_to.py
"""

from ur5e import Robot, Workspace

TARGET = "sim"
HOVER = 0.3      # fraction of the box height
LOW = 0.05


def main() -> None:
    ws = Workspace.load()
    with Robot(target=TARGET, profile=ws) as robot:
        robot.move_joints(ws.home_joints)

        for u, v in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            print(f"corner u={u} v={v} at {HOVER:.0%} height")
            robot.move_to(ws.pose(u, v, HOVER))

        print("centre, slowly")
        robot.move_to(ws.pose(0.5, 0.5, HOVER), speed=0.05, accel=0.2)   # 50 mm/s
        print("down to just above the surface")
        robot.move_to(ws.pose(0.5, 0.5, LOW), speed=0.05)
        print("up")
        robot.move_to(ws.pose(0.5, 0.5, HOVER))

        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()
