"""ur5e — UR5e teaching framework (ZHdK Embodied Interaction HS26).

Student-facing surface: ``Robot``, ``Workspace``, ``grid_poses``, ``line_poses``
(+ ``ur5e.cli`` for the examples' ``--target`` flag) — validated for student use
in simulation (HS26). Real-arm motion stays gated (see motion.py); the examples
open the gate only with an explicit ``--target real``.
"""

from __future__ import annotations

from .connection import RobotConnection
from .config import resolve_host
from .robot import Robot, grid_poses, line_poses
from .workspace import Workspace

__all__ = ["Robot", "Workspace", "grid_poses", "line_poses",
           "RobotConnection", "resolve_host", "connect"]


def connect(target_or_host: str, **kwargs) -> RobotConnection:
    """Connect by named target ('sim'/'real') or a raw host/IP.

    Receive-only unless control=True / io=True are passed through.
    Returns an already-connected RobotConnection.
    """
    host = resolve_host(target_or_host) if target_or_host in ("sim", "real") else target_or_host
    return RobotConnection(host, **kwargs).connect()
