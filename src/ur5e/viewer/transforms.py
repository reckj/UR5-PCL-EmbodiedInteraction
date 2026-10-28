"""Small rigid-transform helpers for the viewer (numpy only).

STATUS: HS26 student framework (sim-validated).

Conventions (match PolyScope / ur_rtde):
- lengths in metres, angles in radians unless a name says ``_deg``
- 4x4 homogeneous matrices, column vectors
- ``rpy`` = roll, pitch, yaw applied as R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
  (the "RPY" convention PolyScope shows in the Move tab, and URDF's)
- ``rotvec`` = axis-angle rotation vector [rx, ry, rz], the default
  orientation representation of PolyScope and of ur_rtde poses
- quaternions are ``wxyz`` (what viser expects)
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

Vec3 = Sequence[float]


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_rot(rpy: Vec3) -> np.ndarray:
    r, p, y = rpy
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def rot_to_rpy(R: np.ndarray) -> np.ndarray:
    """Inverse of rpy_to_rot (returns radians). Gimbal-lock safe enough for UI."""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    p = math.asin(sy)
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: put everything into yaw
        r = 0.0
        y = math.atan2(-R[0, 1], R[1, 1])
    return np.array([r, p, y])


def rotvec_to_rot(rv: Vec3) -> np.ndarray:
    rv = np.asarray(rv, dtype=float)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def rot_to_rotvec(R: np.ndarray) -> np.ndarray:
    q = rot_to_quat_wxyz(R)
    w = max(-1.0, min(1.0, q[0]))
    th = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return np.zeros(3)
    axis = q[1:] / s
    if th > math.pi:  # keep the short representation like PolyScope does
        th -= 2 * math.pi
    return axis * th


def rot_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z). Robust (Shepperd)."""
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_wxyz_to_rot(q: Vec3) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_tf(R: np.ndarray | None = None, t: Vec3 | None = None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=float)
    return T


def tf_from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> np.ndarray:
    return make_tf(rpy_to_rot(rpy), xyz)


def tf_from_xyz_rpy_deg(xyz: Vec3, rpy_deg: Vec3) -> np.ndarray:
    return make_tf(rpy_to_rot(np.radians(rpy_deg)), xyz)


def tf_from_ur_pose(pose: Sequence[float]) -> np.ndarray:
    """UR pose [x, y, z, rx, ry, rz] (m, rotation vector rad) -> 4x4."""
    return make_tf(rotvec_to_rot(pose[3:6]), pose[0:3])


def ur_pose_from_tf(T: np.ndarray) -> np.ndarray:
    return np.concatenate([T[:3, 3], rot_to_rotvec(T[:3, :3])])


def tf_to_viser(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 -> (wxyz, position) as viser wants them."""
    return rot_to_quat_wxyz(T[:3, :3]), T[:3, 3].copy()


def inv_tf(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3].T
    return make_tf(R, -R @ T[:3, 3])
