"""Safety-plane geometry in PolyScope terms — visualization only.

STATUS: HS26 student framework (sim-validated). THIS DOES NOT BLOCK OR LIMIT MOTION. It only
draws where a student's intended planes are and whether the TCP / elbow is on
the wrong side, so the same values can be typed into
Installation > Safety > Planes 1:1. The controller's safety configuration is
the authoritative layer; the project safety model is still being defined.

PolyScope model (UR5e User Manual §21.11):
- a plane copies a *feature*; the feature's **Z axis is the plane normal**
  and its origin, shifted by *displacement* along Z, lies on the plane
- the arrow drawn on the pendant = the normal = the side the TCP (tool
  spheres) and, if "Restrict Elbow" is on, the elbow is **allowed** to be on
- restriction modes: Disabled, Normal, Reduced, Normal & Reduced,
  Trigger Reduced Mode (beyond the plane the safety system switches to
  Reduced mode; it is not a hard limit)
- up to 8 planes; "Restrict Elbow" is a per-plane checkbox

Sign convention assumed here: **allowed side = +Z of the (shifted) feature
frame**, matching the "arrow = allowed side" statement in the manual. To be
confirmed against the pendant's 3D view in the lab (flagged in docs).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from .scene import FEATURE_BASE, FEATURE_TOOL, SafetyPlane, Scene
from .transforms import make_tf, tf_from_xyz_rpy_deg

MAX_PLANES = 8   # PolyScope limit

# colours loosely follow the pendant: yellow/black normal, blue/green trigger, grey disabled
MODE_COLOR = {
    "normal": (250, 200, 30),
    "reduced": (250, 140, 30),
    "normal_and_reduced": (230, 170, 20),
    "trigger_reduced": (40, 170, 200),
    "disabled": (150, 150, 150),
}
VIOLATION_COLOR = (230, 50, 50)
TRIGGERED_COLOR = (120, 90, 210)


def feature_tf(scene: Scene, feature_id: str, frames: Optional[Dict[str, np.ndarray]] = None) -> np.ndarray:
    """World pose of a feature frame. 'base' = identity, 'tool' = live tool0
    (not valid for safety planes on the pendant, kept for completeness)."""
    if feature_id == FEATURE_BASE:
        return np.eye(4)
    if feature_id == FEATURE_TOOL:
        if frames is None:
            raise ValueError("tool feature needs live frames")
        return frames["tool0"]
    f = scene.feature(feature_id)
    if f is None:
        raise ValueError(f"unknown feature {feature_id!r}")
    return tf_from_xyz_rpy_deg(f.pose.xyz, f.pose.rpy_deg)


def plane_tf(scene: Scene, plane: SafetyPlane) -> np.ndarray:
    """World pose of the plane: feature frame shifted by displacement along its Z.
    The plane's +Z is the allowed side."""
    T = feature_tf(scene, plane.feature)
    return T @ make_tf(t=(0.0, 0.0, plane.displacement))


def signed_distance(T_plane: np.ndarray, point_world: np.ndarray) -> float:
    """> 0 : on the allowed (+Z / arrow) side; < 0 : beyond the plane."""
    n = T_plane[:3, 2]
    return float(n @ (np.asarray(point_world) - T_plane[:3, 3]))


@dataclass
class PlaneStatus:
    name: str
    tcp_d: float
    elbow_d: Optional[float]     # None when restrict_elbow is off
    violated: bool               # a strict-limit plane is crossed (normal/reduced/n&r)
    triggered: bool              # a trigger plane is crossed (-> Reduced mode)

    @property
    def color(self):
        return None  # chosen by caller via mode


def evaluate(scene: Scene, frames: Dict[str, np.ndarray], tcp_world: np.ndarray) -> Dict[str, PlaneStatus]:
    """Per-plane crossing status for the current robot state (display only)."""
    out: Dict[str, PlaneStatus] = {}
    elbow = frames["elbow"][:3, 3]
    for p in scene.safety_planes:
        T = plane_tf(scene, p)
        d_tcp = signed_distance(T, tcp_world)
        d_elb = signed_distance(T, elbow) if p.restrict_elbow else None
        crossed = d_tcp < 0 or (d_elb is not None and d_elb < 0)
        violated = crossed and p.mode in ("normal", "reduced", "normal_and_reduced")
        triggered = crossed and p.mode == "trigger_reduced"
        out[p.name] = PlaneStatus(p.name, d_tcp, d_elb, violated, triggered)
    return out


def render_color(plane: SafetyPlane, status: Optional[PlaneStatus]):
    if status is not None and status.violated:
        return VIOLATION_COLOR
    if status is not None and status.triggered:
        return TRIGGERED_COLOR
    return MODE_COLOR.get(plane.mode, (200, 200, 200))
