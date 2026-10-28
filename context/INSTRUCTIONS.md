# Instructions for an AI assistant working in this repository

You are helping a design student (Interaction Design, ZHdK) write Python that
moves a Universal Robots UR5e — first in a simulator, later on the real arm in
the lab. The student is not a robotics engineer; the framework in `src/ur5e/`
exists so that they do not have to be. Follow these rules. They are the
course's conventions, not generic advice.

Read `context/KNOWLEDGE.md` once at the start (facts, API, limits). This file
is about *how* to help.

## 1. Use the framework, nothing else

- Every script uses `from ur5e import Robot, Workspace` and talks to the arm
  through `Robot` methods (`move_to`, `move_joints`, `move_through`,
  `read_digital_in`, `set_digital_out`, …). Do not write `RTDEControlInterface`
  / `rtde_control` / `urx` / URScript code unless the student explicitly asks
  for the raw layer — then say that it bypasses the framework's checks and
  keep it to `target="sim"` (see `examples/function_templates/09_raw_rtde.py`).
- Only the API in KNOWLEDGE §4 exists. **Do not invent methods or
  parameters** (no `Orientation`, no `blend=`, no `approach()`, no
  `robot.freedrive()`, no `degrees=True`, no `read_tool_analog_in`). If
  something is missing, say so and build it from what exists in the
  student's own script, or point them to the instructor.
- Only the packages in the image are available: stdlib, `numpy`, `ur_rtde`,
  `viser`, `trimesh`. Never propose `pip install` (it does not persist).
- Start from the templates: copy the closest file in
  `examples/function_templates/` into `students/<name>/` and modify it. Keep
  the shape of a template: `Workspace.load()` → `with Robot(...) as robot:` →
  `move_joints(ws.home_joints)` first and last.

## 2. Positions come from the taught workspace

- Students describe positions as **fractions of the taught box**
  (`ws.pose(u, v, w)`, `ws.grid(...)`, `ws.line(...)`), not as absolute
  metres. Never hard-code Base-frame coordinates or orientations you have not
  read back from the robot; the only orientation to use is the taught one
  (`ws.orientation`, built into `ws.pose`).
- If the student has no `workspace.json`: `python
  examples/function_templates/00_teach_workspace.py` (add `--demo` for a
  quick simulator box). Do not write a workspace file by hand.
- Pass the workspace to the robot: `Robot(target="sim", profile=ws)`. It is
  what keeps every move inside the box.

## 3. Units and conventions

- Metres and radians everywhere. `move_joints` takes radians — convert
  degrees with `math.radians`, never pass degrees.
- A pose is `[x, y, z, rx, ry, rz]`, Base frame, rotation vector. Joints are
  `[base, shoulder, elbow, wrist1, wrist2, wrist3]`.
- Speeds: the profile ceilings (`sim_open` 0.25 m/s; the real cell 0.15 m/s)
  are the maximum; suggest half of that for anything new. Do not propose
  raising limits, editing `src/ur5e/safety.py`, or passing a custom
  `SafetyLimits`.
- Shell commands for the student: **no trailing `# comments` on a line** —
  macOS zsh passes them to the command (`bash: #: No such file or directory`).
  Put explanations above the command. Commands run inside
  `docker compose run --rm dev bash`, from the repo root.

## 4. Sim first, real arm only with the instructor

- Develop and run everything against the simulator (`target="sim"`). A
  script is ready for the lab when it runs cleanly in the sim end-to-end.
- `target="real"` requires the student to be in the lab with the instructor
  present; the code additionally needs `allow_real=True` and
  `robot.set_tool(payload_kg, tcp)` with the **real** mounted tool's values
  (flag that these must be measured, not guessed), and the host
  `UR_HOST=192.168.1.11`. Never add `allow_real=True` to a script by default,
  never set it to work around a `SafetyViolation`, and do not generate code
  that disables, catches-and-ignores, or works around `SafetyViolation`
  (catching it to *print* the reason, as template 08 does, is fine).
- `SafetyViolation` means the plan was wrong, not the check: fix the pose,
  fraction or speed. The real-arm box is fixed by the lab profile
  (`cart_against_wall`); a taught box can only make it smaller.
- Do not add safety warnings, disclaimers or extra "safety rules" to the
  student's code or to your answers beyond what `docs/` and this file say.
  The course has a defined safety model (PolyScope configuration +
  framework checks); do not invent a second one.

## 5. Simulator realities

- Digital/analog **inputs always read 0 in URSim** — a sensor-gated loop
  will skip every cell. For sim testing, fake the sensor value in the
  student's script (a variable or `random`), clearly marked, and remove the
  fake for the lab. Outputs work and are visible in the pendant's I/O tab.
- `Failed to start RTDE data synchronization` / `control interface not
  connected` → the pendant is not in **Remote Control** or the robot is not
  ON → START. Not a code problem.
- URSim is slow under emulation on Apple Silicon (1–3 min boot). Suggest
  waiting before assuming a connection bug.
- Programs the student writes on the simulated pendant (`programs/*.urp`)
  can be carried to the real pendant on USB (same PolyScope 5.9 line).
  Never copy `default.installation` to the robot.

## 6. How to answer

- Short. Working code over explanation. One script per task, runnable as
  `python students/<name>/<file>.py`. Print what the robot is about to do
  before each move stage (the templates do this) so the student can follow
  along in the viewer (http://localhost:7001/).
- When the student describes a fabrication process (dipping, stamping,
  drawing, scanning, placing…), decompose it into: home → a sequence of
  `ws.pose`/`ws.grid`/`ws.line` targets → a pause or I/O action at each →
  home. Point out where a sensor or output would plug in
  (`read_digital_in`, `set_digital_out`) and what their timing assumptions are
  (`time.sleep` is the only timing primitive; there is no blending, the arm
  stops at every pose).
- State assumptions explicitly: which TCP/tool, which height fraction,
  which speed. If the student's tool changes the TCP, remind them that the
  workspace was taught with the pendant's active TCP at the time and the
  joint check uses the flange position (teach `top`/`home` with headroom).
- Do not edit `src/`, `scripts/`, `examples/`, `docs/`, `docker-compose.yml`
  — they are replaced on `git pull`. Student work lives in `students/<name>/`.
- When unsure about a controller detail (I/O voltages, safety settings,
  pendant menus) consult `context/manual/UR5e_User_Manual_en_Global.txt`;
  for `ur_rtde` call signatures consult `context/ur_rtde/api_reference.txt`
  (C++ names; the Python bindings are identical minus types). Quote the
  section rather than guessing.
