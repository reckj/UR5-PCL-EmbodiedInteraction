"""01 — Read the robot's state: joints() and pose(). No motion.

    robot.joints()  -> [q0..q5]            joint angles in RADIANS
    robot.pose()    -> [x, y, z, rx, ry, rz] TCP position in METRES + rotation
                       vector in RADIANS, robot Base frame
                       (UR: the numbers under Move → TCP, "Rotation Vector")

Also shows where the current pose sits inside the taught workspace, as
fractions (0..1 along x, y, z of the box).

Usage:  python examples/function_templates/01_read_state.py
"""

import math

from ur5e import Robot, Workspace

TARGET = "sim"


def main() -> None:
    ws = Workspace.load()
    print(ws.describe(), "\n")

    with Robot(target=TARGET, profile=ws) as robot:
        q = robot.joints()
        p = robot.pose()

        print("joints [rad]:", [round(v, 3) for v in q])
        print("joints [deg]:", [round(math.degrees(v), 1) for v in q])
        print("pose   [m, rad]:", [round(v, 4) for v in p])
        print("TCP    [mm]:", [round(v * 1000) for v in p[:3]])

        b = ws.box
        frac = [(p[i] - lo) / (hi - lo) for i, (lo, hi) in enumerate((b.x, b.y, b.z))]
        print("inside workspace as fractions (u, v, w):", [round(f, 2) for f in frac],
              "<- 0..1 means inside the box")


if __name__ == "__main__":
    main()
