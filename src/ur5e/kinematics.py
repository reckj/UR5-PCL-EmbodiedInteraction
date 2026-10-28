"""UR5e forward kinematics (DH-based).

Shared by the viewer and the safety/workspace checks. Positions are in the
robot base frame, meters. This computes the FLANGE frame (tool0); a mounted
tool's TCP offset is NOT applied here — callers that care about the true tool
point must account for it, and the authoritative boundary protection is the
robot's PolyScope safety planes, not this.
"""

from __future__ import annotations

import math

import numpy as np

# UR5e DH parameters (meters / radians)
DH_D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
DH_A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]


def _dh(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])


def fk_frames(q):
    """List of 4x4 cumulative transforms: base + each of the 6 joint frames."""
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(6):
        T = T @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        frames.append(T.copy())
    return frames


def fk_origins(q):
    """3xN float32 array of frame origins (base + 6 joints) — for the viewer."""
    return np.array([T[:3, 3] for T in fk_frames(q)]).T.astype(np.float32)


def fk_tcp_position(q):
    """xyz position of the flange (tool0) in the base frame."""
    return fk_frames(q)[-1][:3, 3]
