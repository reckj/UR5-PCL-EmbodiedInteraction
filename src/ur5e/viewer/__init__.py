"""ur5e.viewer — browser 3D viewer + scene editor (viser).

STATUS: HS26 student framework (sim-validated). Visualization only; never part of the
motion or safety path. Entry point: ``scripts/rtde_viewer.py`` ->
:func:`ur5e.viewer.server.run_viewer`.

Modules
- robot_model   UR5e link chain + meshes (ur_description 4.4.0, BSD-3)
- scene         scene.json data model + load/save (scenes/<name>/)
- safety_planes PolyScope-style plane geometry + crossing status (display only)
- server        viser app: GUI, gizmos, RTDE receive loop
- transforms    small numpy rigid-transform helpers

Importing this package does NOT import ur_rtde or viser; those load lazily in
server.py so scene files can be read/written anywhere.
"""
