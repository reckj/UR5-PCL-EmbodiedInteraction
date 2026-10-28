# function_templates/ — one runnable example per API call

Short scripts (≈ 30 lines each) that show one thing. Run them in order once,
then copy whatever you need into `students/<you>/`.

| Template | Shows | UR name |
|---|---|---|
| `00_teach_workspace.py` | teach the work area in 4 steps → `workspace.json` | — |
| `01_read_state.py` | `joints()`, `pose()` — no motion | — |
| `02_move_joints.py` | `move_joints(q, speed, accel)` | moveJ |
| `03_move_to.py` | `move_to(pose, speed, accel)`, `ws.pose(u, v, w)` | moveL |
| `04_move_through.py` | `move_through(poses)`, `ws.line(...)` | moveL ×n |
| `05_grid_poses.py` | `ws.grid(nx, ny, w)` + your own per-cell loop | — |
| `06_digital_io.py` | `read_digital_in`, `set_digital_out`, `set_tool_digital_out` | I/O |
| `07_analog_io.py` | `read_analog_in`, `set_analog_out` | I/O |
| `08_safety_rejections.py` | what a rejected move looks like, `SafetyViolation` | — |
| `09_raw_rtde.py` | the `ur_rtde` objects underneath (`conn.receive/control/io`) — sim only | moveJ raw |

## The workspace: teach once, every example scales to it

All templates (and your own scripts) move inside a **taught box**. You
teach it once per setup with `00_teach_workspace.py` — four points, the
script only reads where the arm is, you move it (sim: pendant **Move** tab;
real robot: **freedrive button** on the pendant, supervised):

1. **corner A** — one corner of the work area, tool on the surface
2. **corner B** — the diagonally opposite corner, on the surface
3. **top** — the highest point the tool should reach
4. **home** — a comfortable pose above the area; every example starts and
   ends here (`robot.move_joints(ws.home_joints)`)

Result: `workspace.json` in the repo folder (git-ignored, one per setup).
In a hurry in the sim: `--demo` writes a 300 × 300 × 200 mm box around the
sim's start pose instead.

```python
from ur5e import Robot, Workspace

ws = Workspace.load()                      # workspace.json
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)      # taught home
    robot.move_to(ws.pose(0.5, 0.5, 0.3))  # centre of the area, 30 % of the height
    for cell in ws.grid(3, 3, w=0.1):      # 3 x 3 over the area, 10 % up
        robot.move_to(cell)
```

`ws.pose(u, v, w)` takes **fractions 0…1** of the box along x, y, z —
`(0, 0, 0)` is the low corner on the surface, `(1, 1, 1)` the far top. The
orientation is the one you taught at corner A (tool pointing down). Because
everything is relative to the box, the same script runs on a 300 mm demo
box in the sim and on the 600 × 500 mm cart in the lab.

`Robot(profile=ws)` turns the box into the safety profile for that run: moves
outside it are rejected before they are sent. On the **real robot** the box
is additionally clipped to the measured cell (`cart_against_wall`) and keeps
its speed ceilings — you can make the space smaller, never larger. A
workspace taught in the sim is refused on the real robot: teach it again
there (two minutes with freedrive).

Flagged assumptions: poses are TCP poses with the pendant's active TCP at
teaching time; with a long tool, teach `top` and `home` with extra headroom
(the joint check looks at the flange). The simulator's inputs always read 0.
