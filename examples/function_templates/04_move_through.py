"""04 — move_through(): a sequence of straight-line moves (a path).

    robot.move_through(poses, speed=None, accel=None)
        poses  any list of poses — the robot does move_to() for each one and
               STOPS at every point (so you can read a sensor or act there)

Paths come from generators that only compute geometry:
    ws.line((u0, v0, w0), (u1, v1, w1), n)   n poses between two fractional points
    ur5e.line_poses(start_pose, end_xyz, n)  the same with absolute poses

What this does: home → a diagonal line across the area in 6 steps →
a zig-zag back along the other edge → home.

Usage:  python examples/function_templates/04_move_through.py
"""

from ur5e import Robot, Workspace

TARGET = "sim"
H = 0.2   # height fraction for the whole path


def main() -> None:
    ws = Workspace.load()
    with Robot(target=TARGET, profile=ws) as robot:
        robot.move_joints(ws.home_joints)

        diagonal = ws.line((0.1, 0.1, H), (0.9, 0.9, H), n=6)
        print(f"diagonal, {len(diagonal)} points")
        robot.move_through(diagonal)

        zigzag = [ws.pose(0.9, 0.1, H), ws.pose(0.5, 0.5, H + 0.2), ws.pose(0.1, 0.9, H)]
        print("zig-zag with a hop in the middle, slowly")
        robot.move_through(zigzag, speed=0.08)

        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()
