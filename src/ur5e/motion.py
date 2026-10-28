"""Validated motion primitives.

STATUS: HS26 student framework, lower layer (sim-validated). Students use ur5e.Robot.

Wraps a connected RobotConnection with a SafetyLimits profile. Every motion is
validated against the profile and REJECTED (raises SafetyViolation) on any
violation — never silently clamped, so mistakes surface.

Real-arm gate: motion to a non-sim host requires BOTH allow_real=True AND a
validated profile AND (if the profile requires) payload + TCP already set.
Sim motion runs the geometry/speed checks but skips the real-arm gate.

These software checks are SECONDARY to the robot's PolyScope Safety
Configuration, which is the authoritative protection. See safety.py.
"""

from __future__ import annotations

from . import config
from .connection import RobotConnection
from .safety import SafetyLimits, SafetyViolation


class MotionController:
    def __init__(self, conn: RobotConnection, limits: SafetyLimits, *, allow_real: bool = False):
        if conn.control is None:
            raise RuntimeError("RobotConnection has no control interface (construct with control=True)")
        self.conn = conn
        self.limits = limits
        self.allow_real = allow_real
        self.is_sim = config.is_sim(conn.host)

    def _preflight(self) -> None:
        if not self.conn.connected:
            raise RuntimeError("not connected")
        if self.is_sim:
            return
        # Real-arm gates.
        if not self.allow_real:
            raise SafetyViolation(
                f"refusing real-arm motion to {self.conn.host}: construct "
                "MotionController(..., allow_real=True) to permit hardware motion")
        if not self.limits.validated:
            raise SafetyViolation(
                f"profile '{self.limits.name}' is not validated; confirm cell "
                "geometry (and PolyScope config) and set validated=True first")
        if self.limits.require_payload and not self.conn.payload_set:
            raise SafetyViolation("payload not set; call conn.set_payload(...) before real motion")
        if self.limits.require_tcp and not self.conn.tcp_set:
            raise SafetyViolation("TCP not set; call conn.set_tcp(...) before real motion")

    def move_j(self, q_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.joint_speed_max if speed is None else speed
        accel = self.limits.joint_accel_max if accel is None else accel
        self.limits.check_joint_speed(speed, accel)
        self.limits.check_joint_target(q_target)
        return self.conn.control.moveJ(list(q_target), speed, accel)

    def move_l(self, pose_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.tcp_speed_max if speed is None else speed
        accel = self.limits.tcp_accel_max if accel is None else accel
        self.limits.check_tcp_speed(speed, accel)
        self.limits.check_pose_target(pose_target)
        return self.conn.control.moveL(list(pose_target), speed, accel)
