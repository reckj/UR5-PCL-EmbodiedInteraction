# UR5e visual meshes — source and license

**Source:** Universal Robots, `Universal_Robots_ROS2_Description`
(https://github.com/UniversalRobots/Universal_Robots_ROS2_Description),
package `ur_description` version **4.4.0**, commit
`81f7c9fe003c2f0e02eec5a7253f1b262ad38b61` (2026-10-22).
Files: `meshes/ur5e/visual/{base,shoulder,upperarm,forearm,wrist1,wrist2,wrist3}.dae`.

**License:** BSD-3-Clause (see `LICENSE` next to this file, copied verbatim from
the repository). The UR5e meshes are under this BSD license; only the UR20/UR30
meshes in that repository carry Universal Robots' separate "Terms and Conditions
for use of Graphical Documentation", which do not apply here.

**Conversion (done once, offline, 2026-10-28):** each DAE was loaded with
`trimesh` 5.2.0 + `pycollada`, scene-graph transforms applied (the DAEs are
authored in millimetres / Y-up and carry a 0.001 scale + Z-up node transform),
vertices merged, geometries with > 2000 faces decimated by 50 % with
`fast-simplification`, per-geometry material colours baked as vertex colours,
and exported as binary glTF (`.glb`). Result: 9.4 MB → 1.4 MB, bounding boxes
identical to the originals to 0.1 mm. Geometry is in **metres, Z-up, in each
link's frame** exactly as the original DAE (the URDF `mesh_offset` from
`config/ur5e/visual_parameters.yaml` is applied in `robot_model.py`, not baked
into the files).

These are nominal visual meshes of a UR5e. They do not include any per-robot
kinematic calibration.
