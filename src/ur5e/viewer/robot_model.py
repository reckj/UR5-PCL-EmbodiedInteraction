"""UR5e link chain + visual meshes for the viewer.

STATUS: HS26 student framework (sim-validated). Visualization only — nothing here is used for
motion or safety decisions.

The chain below is the UR5e model from Universal Robots' own
``ur_description`` (ROS 2, 4.4.0): joint origins from
``config/ur5e/default_kinematics.yaml`` and per-link mesh offsets from
``config/ur5e/visual_parameters.yaml``. It is rooted directly in the
**controller Base frame** (``ur_description``'s ``base`` link = ``base_link``
rotated 180 deg about Z), so everything the viewer shows is in the frame
students read on the pendant and get from ``getActualTCPPose()``.

Agreement with the framework's DH forward kinematics (``ur5e.kinematics``)
is checked by ``scripts/verify_viewer_fk.py``: both must give the same flange
pose for any q. The DH FK remains the reference used by the safety layer.

Frames exposed by :func:`fk`:
    base, j1..j6 (frame *after* each joint rotation), flange, tool0
where ``tool0`` is UR's tool-flange frame (Z out of the flange) = the frame
``getActualTCPPose()`` reports when the TCP offset is zero. ``elbow`` is the
origin of ``j3`` — what PolyScope's "Restrict Elbow" refers to.

Assumptions: nominal UR5e geometry (no per-robot calibration); meshes are
visual only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Sequence

import numpy as np

from .transforms import make_tf, rot_z, tf_from_xyz_rpy

ASSET_DIR = Path(__file__).parent / "assets" / "ur5e"
PI = math.pi

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")

# PolyScope "Home" pose (joints, rad): upright, elbow up. Used offline only.
HOME_Q = (0.0, -PI / 2, 0.0, -PI / 2, 0.0, 0.0)


@dataclass(frozen=True)
class LinkSpec:
    name: str
    mesh: str                       # file in ASSET_DIR
    mesh_xyz: tuple                 # visual origin (URDF mesh_offset)
    mesh_rpy: tuple
    joint_xyz: tuple = (0.0, 0.0, 0.0)   # origin of the joint that moves this link,
    joint_rpy: tuple = (0.0, 0.0, 0.0)   # expressed in the parent link frame


# Order = kinematic chain. The first entry is the fixed base.
LINKS: Sequence[LinkSpec] = (
    LinkSpec("base", "base.glb", (0, 0, 0), (0, 0, PI)),
    LinkSpec("shoulder", "shoulder.glb", (0, 0, 0), (0, 0, PI),
             joint_xyz=(0, 0, 0.1625)),
    LinkSpec("upper_arm", "upperarm.glb", (0, 0, 0.138), (PI / 2, 0, -PI / 2),
             joint_rpy=(PI / 2, 0, 0)),
    LinkSpec("forearm", "forearm.glb", (0, 0, 0.007), (PI / 2, 0, -PI / 2),
             joint_xyz=(-0.425, 0, 0)),
    LinkSpec("wrist_1", "wrist1.glb", (0, 0, -0.127), (PI / 2, 0, 0),
             joint_xyz=(-0.3922, 0, 0.1333)),
    LinkSpec("wrist_2", "wrist2.glb", (0, 0, -0.0997), (0, 0, 0),
             joint_xyz=(0, -0.0997, 0), joint_rpy=(PI / 2, 0, 0)),
    LinkSpec("wrist_3", "wrist3.glb", (0, -0.0005, -0.0989), (PI / 2, 0, 0),
             joint_xyz=(0, 0.0996, 0), joint_rpy=(PI / 2, PI, PI)),
)

# wrist_3_link -> flange -> tool0 (fixed), from ur_macro.xacro
T_W3_FLANGE = tf_from_xyz_rpy((0, 0, 0), (0, -PI / 2, -PI / 2))
T_FLANGE_TOOL0 = tf_from_xyz_rpy((0, 0, 0), (PI / 2, 0, PI / 2))


def fk(q: Sequence[float]) -> Dict[str, np.ndarray]:
    """World (= controller base) pose of every frame for joint vector q (rad)."""
    q = np.asarray(q, dtype=float)
    assert q.shape == (6,), "q must have 6 joint angles (rad)"
    frames: Dict[str, np.ndarray] = {"base": np.eye(4)}
    T = np.eye(4)
    for i, link in enumerate(LINKS[1:], start=1):
        T = T @ tf_from_xyz_rpy(link.joint_xyz, link.joint_rpy) @ make_tf(rot_z(q[i - 1]))
        frames[f"j{i}"] = T
        frames[link.name] = T
    frames["flange"] = T @ T_W3_FLANGE
    frames["tool0"] = frames["flange"] @ T_FLANGE_TOOL0
    frames["elbow"] = frames["j3"]
    return frames


def link_world_poses(frames: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """World pose of each link's *mesh* (link frame @ mesh offset)."""
    out = {}
    for link in LINKS:
        T_link = frames["base"] if link.name == "base" else frames[link.name]
        out[link.name] = T_link @ tf_from_xyz_rpy(link.mesh_xyz, link.mesh_rpy)
    return out


def mesh_bytes(link: LinkSpec) -> bytes:
    return (ASSET_DIR / link.mesh).read_bytes()
