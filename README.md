# UR5e — Embodied Interaction & Fabrication HS26

Python framework and simulator setup for the UR5e cobot course at ZHdK
Interaction Design. You program a **simulated UR5e** on your own laptop with
the exact same code that later runs on the real arm in the lab.

**→ [docs/SETUP.md](docs/SETUP.md)** — install Docker Desktop, VS Code, Git;
first start; checks. Do this before the first session.

**→ [docs/USAGE.md](docs/USAGE.md)** — the Python API, examples, sim vs. real
robot, the 3D viewer (scenes, models, toolheads, safety planes), PolyScope
programs → USB → pendant.

## In short

Start the simulator and the 3D viewer:

```bash
docker compose up -d ursim viewer
```

Open http://localhost:6080/vnc.html (Connect → power button → ON → START →
header icon: Remote Control) and http://localhost:7001/ (live 3D view). Then
open a Python shell inside the container and run things:

```bash
docker compose run --rm dev bash
python scripts/check_connection.py --target sim
python examples/grid_process.py
```

**Simulator = the lab robot's software version.** The simulator in
`docker-compose.yml` is pinned to the PolyScope version of the real UR5e
(`universalrobots/ursim_e-series:5.9.4` — the robot runs 5.9.1, same 5.9 line),
so programs you save in the simulator load on the real pendant. On the pendant
the version and the robot's IP are under **☰ (top right) → About**:
*URSoftware 5.9.1…* and *IP address 192.168.1.11*. The IP is static — that is
what you put into `UR_HOST` in the lab (see [docs/USAGE.md](docs/USAGE.md) §4).
Only the instructor changes the pinned version, after a PolyScope update.

`[screenshot: pendant ☰ → About — version and IP address]`

```python
from ur5e import Robot, grid_poses

with Robot(target="sim") as robot:
    for cell in grid_poses(robot.pose(), nx=3, ny=3, dx=0.04, dy=0.04):
        robot.move_to(cell)
        if robot.read_digital_in(0):
            robot.set_digital_out(0, True)
            robot.set_digital_out(0, False)
```

## What's in the repo

```
docs/            SETUP.md, USAGE.md
src/ur5e/        the framework: Robot API, connection, motion checks, viewer
scripts/         check_connection.py, smoke_test_move.py, rtde_viewer.py
examples/        grid_process.py — copy it to start your own
scenes/          viewer scenes (scene.json + models/)
toolheads/       toolhead presets for the viewer (mesh + TCP)
programs/        where the simulator saves PolyScope programs (.urp) — git-ignored
students/        your work: students/<your-name>/ — git-ignored
docker-compose.yml   ursim (simulator) · dev (Python shell) · viewer (3D)
Start UR5e Sim.command / .bat   double-click launchers (optional)
```

Nothing is installed on your laptop besides Docker Desktop, VS Code and Git —
Python, `ur_rtde` and the simulator run in containers
(`ghcr.io/reckj/ur5e-dev`, `universalrobots/ursim_e-series`).

Updates: `git pull`, then `docker compose down && docker compose up -d ursim viewer`.
