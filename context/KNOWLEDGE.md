# UR5e framework — knowledge base for AI assistants

Facts about this repository, the Python framework, the simulator and the lab
robot. Written for an AI assistant (any model) helping a student; also fine
for humans. Everything here reflects the code as it is — when this file and
the code disagree, the code wins (`context/framework_source.md` is a
generated snapshot of it).

Course: Embodied Interaction & Fabrication HS26, ZHdK Interaction Design.
Robot: Universal Robots **UR5e** (e-Series, 850 mm reach, 5 kg payload).
Language: Python 3.12. Comms: `ur_rtde` 1.6.3 (RTDE protocol) — identical
against the simulator (URSim) and the real arm; only the host differs.

---

## 1. Environment (what runs where)

Everything runs in Docker containers; nothing is installed on the laptop
except Docker Desktop, VS Code and Git. `docker-compose.yml` defines:

| service | image | role |
|---|---|---|
| `ursim` | `universalrobots/ursim_e-series:5.9.4` (amd64, emulated on Apple Silicon) | the simulated controller + pendant UI (PolyScope). Pinned to the real robot's PolyScope line (5.9.x). |
| `dev` | `ghcr.io/reckj/ur5e-dev:hs26.1` (amd64 + arm64) | Python 3.12, `ur_rtde` 1.6.3, `viser` 1.1.1, `trimesh` 5.2.0, `numpy`. The repo is bind-mounted at `/workspace`; the `ur5e` package is installed editable on start. |
| `viewer` | same image | browser 3D viewer at http://localhost:7001/ (receive-only) |

Ports on the laptop: **6080** pendant UI (`http://localhost:6080/vnc.html`),
**7001** viewer, 29999 dashboard, 30001–30004 RTDE/primary/secondary.
Inside the containers the simulator's hostname is `ursim` (`URSIM_HOST=ursim`).

Daily routine (type commands **without** trailing `# comments` — macOS zsh
passes them to the command):

```bash
docker compose up -d ursim viewer
docker compose run --rm dev bash
python examples/function_templates/03_move_to.py
exit
docker compose down
```

Inside `docker compose run --rm dev bash` you are root in `/workspace`; files
edited on the laptop are live. **`pip install` inside the container is lost
when the shell exits** — only `numpy` + stdlib + `ur_rtde` + `viser`/`trimesh`
are available. Do not propose extra packages.

Simulator must be **ON → START** (power button bottom-left of the pendant)
and in **Remote Control** (header icon; enable once under ☰ → Settings →
System → Remote Control) before any motion script works. Local mode →
`RuntimeError: Failed to start RTDE data synchronization`. Reading state
(receive-only) works in both modes. First boot shows a "Confirmation of
applied Safety Configuration" dialog → Confirm.

URSim start pose: joints ≈ (−92, −99, −126, −46, 91, −2)°, TCP ≈
(−144, −436, 202) mm, tool pointing down.

**The simulator cannot drive inputs:** `read_digital_in` / `read_analog_in`
always return 0 in URSim (I/O-tab input boxes are read-only; the pendant's
*Simulation* toggle does not reach RTDE). Outputs do work and show in the I/O
tab. To test a sensor branch in the sim, fake the value in code temporarily.

## 2. Repository layout

```
docker-compose.yml          services above
src/ur5e/                   the framework (importable as `ur5e`)
scripts/                    check_connection.py  smoke_test_move.py  rtde_viewer.py
examples/grid_process.py    grid traversal with sensor-gated actuation
examples/function_templates/   00_teach_workspace … 09_raw_rtde — one script per API call (see its README)
scenes/<name>/              viewer scenes (scene.json + models/)
toolheads/<id>/             toolhead presets for the viewer (toolhead.json + mesh)
programs/                   where URSim saves PolyScope programs (.urp) — git-ignored
students/<name>/            the student's own scripts — git-ignored
workspace.json              the taught workspace (see §5) — git-ignored
docs/SETUP.md, docs/USAGE.md   human docs
context/                    this folder
```

Run scripts from the repo root inside the dev shell:
`python students/anna/my_script.py`. Students copy from `examples/`, never
edit `src/`, `scripts/`, `examples/` (replaced on `git pull`).

## 3. Units, frames, conventions

- Lengths **metres**, angles **radians** — also in function arguments
  (`move_joints` takes radians, not degrees).
- A **pose** is `[x, y, z, rx, ry, rz]`: position in the robot **Base** frame
  + orientation as a **rotation vector** (axis × angle, rad) — exactly what
  the pendant shows under Move → TCP when set to "Rotation Vector [rad]".
  Tool pointing straight down ≈ `rx, ry, rz ≈ (0, π, 0)` (URSim start:
  `(-0.0012, 3.1163, 0.0389)`).
- Joint vector `q = [base, shoulder, elbow, wrist1, wrist2, wrist3]`, rad.
- The TCP is the pendant's **active TCP**; with no tool mounted it is the
  flange (`tool0`). `Robot.set_tool(payload_kg, tcp)` sets payload + TCP for a
  session; required on the real arm.
- Base frame on the lab cart: table surface at z ≈ 0, **front of the robot
  = −X** (the measured work area is at x ≈ −0.49…−0.20 m).

## 4. The `ur5e` package — public API

```python
from ur5e import Robot, Workspace, grid_poses, line_poses
from ur5e.safety import SafetyViolation          # the rejection exception
```

### `Robot` (src/ur5e/robot.py) — the student-facing class

```python
Robot(target="sim", *, profile=None, allow_real=False, host=None)
```
- `target`: `"sim"` (host from `URSIM_HOST`, default 127.0.0.1) or `"real"`
  (host from `UR_HOST`). `host=` overrides.
- `profile`: `None` → `"sim_open"` for sim, `"cart_against_wall"` for real;
  a profile name; a `SafetyLimits`; or a **`Workspace`** (→ `ws.profile(target)`).
- Use as a context manager: `with Robot(...) as robot:` connects
  (receive + control + io) and disconnects cleanly, also on exceptions.

| method | does | units / notes |
|---|---|---|
| `robot.joints()` | current joint angles | list of 6, rad |
| `robot.pose()` | current TCP pose | `[x,y,z,rx,ry,rz]` m, rad |
| `robot.move_joints(q, speed=None, accel=None)` | joint-space move (UR moveJ) | q rad; speed rad/s, accel rad/s²; default = profile max |
| `robot.move_to(pose, speed=None, accel=None)` | linear TCP move (UR moveL) | speed m/s, accel m/s²; default = profile max |
| `robot.move_through(poses, speed=None, accel=None)` | `move_to` for each pose, **stops at every point** | no blending |
| `robot.set_tool(payload_kg, tcp=(0,0,0,0,0,0))` | declare payload + TCP offset | required before real-arm motion |
| `robot.read_digital_in(pin) -> bool` | standard digital input 0–7 | always False in sim |
| `robot.read_analog_in(pin) -> float` | standard analog input 0 or 1, volts | 0.0 in sim |
| `robot.set_digital_out(pin, value)` | standard digital output 0–7 | visible in pendant I/O tab |
| `robot.set_tool_digital_out(pin, value)` | tool-flange output 0 or 1 | |
| `robot.set_analog_out(pin, voltage)` | standard analog output 0 or 1 | |
| `robot.conn` | the `RobotConnection` underneath | `.receive` `.control` `.io` = raw ur_rtde objects |
| `robot.limits` | the active `SafetyLimits` | |

Every move is **validated before it is sent**: target inside the profile's
workspace box (`move_to`: the target xyz; `move_joints`: the flange position
from forward kinematics), speed/accel ≤ ceilings, joints within ±2π. A
violation raises `SafetyViolation` with a readable message and **nothing
moves**; values are never clamped.

### Pattern generators (geometry only, no motion)

```python
grid_poses(origin, nx, ny, dx, dy)   # nx*ny poses from an origin pose, steps dx/dy in base X/Y (m),
                                     # origin's z + orientation, snake order
line_poses(start, end_xyz, n)        # n poses from a full pose to an xyz, start's orientation
```

### `Workspace` (src/ur5e/workspace.py) — the taught box (§5)

```python
ws = Workspace.load()                   # workspace.json (or $UR5E_WORKSPACE / a path)
ws.pose(u, v, w)                        # fractions 0..1 of the box along x, y, z -> pose (taught orientation)
ws.grid(nx, ny, w=0.1, margin=0.05)     # poses over the area at height fraction w, snake order
ws.line(a_uvw, b_uvw, n)                # n poses between two fractional points
ws.home_joints                          # taught home, rad  -> robot.move_joints(ws.home_joints)
ws.home, ws.corner_a, ws.corner_b, ws.top   # the taught poses
ws.box                                  # Box(x=(lo,hi), y=(..), z=(..)) in metres, Base frame
ws.orientation                          # rotation vector taught at corner A
ws.profile(target)                      # SafetyLimits for this box (real: clipped to cart_against_wall)
ws.describe(); ws.size_mm; ws.target    # "sim" or "real"
Workspace.from_points(target, a, b, top, home, home_joints); ws.save(path)
```
`WorkspaceError` if the file is missing/invalid or taught on the other target.

### Lower layers (students normally do not touch)

- `ur5e.connection.RobotConnection(host, *, frequency=-1.0, control=False, io=False)`
  — `.receive` (RTDEReceiveInterface), `.control` (RTDEControlInterface),
  `.io` (RTDEIOInterface); `set_payload(mass, cog)`, `set_tcp(pose6)`;
  `disconnect()` stops the control script.
- `ur5e.motion.MotionController(conn, limits, allow_real=False)` —
  `move_j`, `move_l`; holds the real-arm gate.
- `ur5e.safety` — `SafetyLimits`, `Box`, `PROFILES`, `get_profile(name)`,
  `SafetyViolation`.
- `ur5e.config` — `resolve_host("sim"|"real")`, `is_sim(host)`.
- `ur5e.kinematics` — DH forward kinematics (flange).
- `ur5e.viewer` — the viser-based viewer (scenes, safety-plane drawing).

### Not implemented (do not invent these)

No `Orientation` class or named orientation presets; no blend radius /
`blend=` on `move_through`; no `approach`/`retract`/`traverse`/`probe`; no
freedrive API or waypoint recording/save-load in `Robot`; no
`read_tool_analog_in`; no servo/speed loops in `Robot` (only via
`robot.conn.control.servoJ/speedL…`, raw, unchecked). Degrees are never
accepted — convert with `math.radians`.

## 5. The taught workspace (`workspace.json`)

Taught once per setup with `examples/function_templates/00_teach_workspace.py`
(receive-only: the student moves the arm — sim: pendant Move tab; real:
freedrive button on the pendant — and presses Enter four times):
**corner A** (surface), **corner B** (diagonally opposite, surface), **top**
(highest working point), **home** (comfortable pose above the area; pose and
joint angles stored). Box = bounding box of the four points; orientation =
corner A's. `--demo` writes a 300 × 300 × 200 mm box around the URSim start
pose. Format `ur5e-workspace/1`, JSON, metres/rad.

Canonical script skeleton:

```python
from ur5e import Robot, Workspace

ws = Workspace.load()
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)
    for cell in ws.grid(3, 3, w=0.1):
        robot.move_to(cell)
        # read a sensor / act here
    robot.move_joints(ws.home_joints)
```

`Robot(profile=ws)` makes the box the safety profile: in the sim with
`sim_open`'s speed guards; on the real arm the box is **intersected with
`cart_against_wall`** and that profile's ceilings apply. A sim-taught file is
refused for the real arm (`WorkspaceError`).

## 6. Safety profiles (src/ur5e/safety.py) — numbers

| profile | tcp speed / accel max | joint speed / accel max | workspace box (m, Base) | payload max | real-arm requirements |
|---|---|---|---|---|---|
| `sim_open` (sim default) | 0.25 m/s, 0.8 m/s² | 0.8 rad/s, 1.2 rad/s² | x ±0.9, y ±0.9, z −0.25…1.0 | 0.5 kg | — (not validated) |
| `cart_against_wall` (real default) | 0.15 m/s, 0.5 m/s² | 0.5 rad/s, 0.8 rad/s² | x −0.4853…−0.2013, y −0.2859…0.2401, z 0.0287…0.2337 (captured on the real cell) | 0.5 kg | `allow_real=True`, `validated=True` ✓, payload set, TCP set |

Joint limits ±2π on all joints. The **real-arm gate** (`MotionController`):
motion to a non-sim host needs all of `allow_real=True`, a validated
profile, `set_payload` and `set_tcp` done — otherwise `SafetyViolation`.
Software limits are deliberately **below** the controller's: PolyScope on
the lab robot is set to the "most restrictive" preset (tool speed 250 mm/s,
tool force 100 N, power 80 W, momentum 5 kg·m/s) plus a wall plane and a
table plane. **PolyScope's Safety Configuration is the authoritative layer;
the framework checks are a secondary guard. Nothing in the viewer protects
the robot.**

## 7. The lab robot

- UR5e on a steel cart against a wall (~180° usable), controller in the
  cart, work surface ~600 × 500 mm in front at mounting height (z ≈ 0).
- PolyScope **5.9.1** (hence URSim 5.9.4). Static IP **192.168.1.11**
  (pendant ☰ → About). Remote Control enabled. In the lab:
  `docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash` and
  `Robot(target="real", allow_real=True)` + `robot.set_tool(...)` — only with
  the instructor present, speed slider down, hand near the e-stop.
- No tool mounted by default (payload 0, TCP 0). A `tool_example` preset
  exists in `toolheads/` (80 mm flange plate + probe, TCP ≈ (0, 21, 108) mm —
  assumed from geometry, not measured).
- Tool flange connector (8-pin, Lumberg RKMV 8-354): 2 digital in, 2 digital
  out, 2 analog in (or RS485 instead of the analog pair), power 0/12/24 V
  selected on the pendant (Installation → Tool I/O); ≤ 1 A typ. Digital
  inputs are PNP, floating = LOW, high ≥ 5.5 V. Controller I/O: 8 standard
  DI/DO (24 V), 2 analog in/out. Details: `context/manual/` (user manual
  §5 Electrical Interface, §18 Freedrive, §21 Safety Configuration, §26 I/O Tab).
- Programs written on the sim pendant land in `programs/*.urp` and can be
  carried to the real pendant on USB (same 5.9 line). Never copy
  `default.installation` / `.variables` from the sim to the robot.

## 8. Viewer (http://localhost:7001/)

Renders the arm from live RTDE joint state (sim or real), Base/tool0/TCP
frames, 10 cm grid. Panels: Robot (connection, joints, controller vs scene
TCP check), Scene file (load/save/save-as; `scenes/<name>/scene.json` +
`models/`), Objects (upload STL/OBJ/GLB/PLY, units mm or m, gizmo),
Toolhead + TCP (TCP in PolyScope's numbers; presets from `toolheads/`),
Features + Safety planes (PolyScope vocabulary: feature, displacement,
restriction mode, restrict elbow; a plane turns red when crossed —
**display only, nothing is enforced**). `--no-robot` runs it with joint
sliders. Scene format `ur5e-scene/1` (metres, degrees unless `_rad`).

## 9. Typical errors → cause

| message | cause / fix |
|---|---|
| `bash: #: No such file or directory` | a shell line was pasted with a trailing `# comment`; retype without it |
| `Failed to start RTDE data synchronization` | pendant not in Remote Control (or robot not ON/STARTED) |
| `control interface not connected` | same; or URSim still booting (1–3 min on Apple Silicon) |
| `SafetyViolation: target (...) outside workspace box of profile '...'` | pose outside the box — check fractions / the taught box (`ws.describe()`) |
| `SafetyViolation: tcp speed X out of (0, 0.25] m/s` | speed above the ceiling; use ≤ profile max |
| `SafetyViolation: joint 1 = ... outside limit` | joint target beyond ±2π (degrees passed as radians?) |
| `SafetyViolation: refusing real-arm motion ... allow_real=True` | real arm without `allow_real=True` (deliberate) |
| `WorkspaceError: no workspace file` | run `00_teach_workspace.py` (`--demo` for a quick sim box) |
| `WorkspaceError: taught on target 'sim' but ... 'real'` | teach again on the real setup |
| `connection refused` on port 30004 | URSim not started / not ON |
| grid cells all `skip` in `grid_process.py` | sim inputs read 0 — expected |
| `Not an Installation File` on URSim boot | `programs/` holds a `default.installation` from a newer URSim — empty the folder |
