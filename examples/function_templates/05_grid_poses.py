"""05 — grid: visit every cell of a grid over the work area.

    ws.grid(nx, ny, w=0.1, margin=0.05)
        nx, ny  cells along x and y
        w       height fraction for all cells
        margin  fraction kept free at the edges
        -> list of poses in snake order (row by row, alternating direction)

    ur5e.grid_poses(origin, nx, ny, dx, dy)   the same from an absolute origin
                                               pose with metric spacing

The loop is yours: move, do something, move on. That is the pattern every
"process over an area" starts from — replace do_something() with yours.

What this does: home → 3 x 4 grid at 10 % height, dwell at each cell → home.

Usage:  python examples/function_templates/05_grid_poses.py
"""

import time

from ur5e import Robot, Workspace

TARGET = "sim"
NX, NY = 3, 4


def do_something(robot: Robot, k: int) -> None:
    """Your per-cell process. Here: just wait a moment and print."""
    print(f"  cell {k}: at {[round(v * 1000) for v in robot.pose()[:3]]} mm")
    time.sleep(0.3)


def main() -> None:
    ws = Workspace.load()
    cells = ws.grid(NX, NY, w=0.1)
    print(f"{len(cells)} cells ({NX} x {NY})")

    with Robot(target=TARGET, profile=ws) as robot:
        robot.move_joints(ws.home_joints)
        robot.move_to(ws.pose(0.5, 0.5, 0.5))          # come in from above
        for k, cell in enumerate(cells):
            robot.move_to(cell)
            do_something(robot, k)
        robot.move_to(ws.pose(0.5, 0.5, 0.5))
        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()
