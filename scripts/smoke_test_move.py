"""Control-interface smoke test — SIMULATION ONLY.

STATUS: HS26 student framework — sim-only smoke test (sim-validated).

Confirms RTDEControlInterface can connect and command a small joint-space move
against URSim, then return to start. Uses receive + control.

DO NOT run against the real arm. This deliberately carries the minimal
assumptions appropriate only for a simulated robot:
  - No payload set (setPayload not called) -> dynamics/force not modeled for a
    real tool; acceptable only for a low-speed sim joint move.
  - No TCP set -> this is a joint-space move, so TCP offset doesn't affect the
    outcome; any later Cartesian/force use WILL need a real TCP.
  - Workspace / self-collision NOT checked -> the small joint delta is assumed
    safe in sim only; unverified on hardware.
  - SPEED/ACCEL below are gentle smoke-test values, NOT a safety model.

Precondition: URSim powered ON and STARTED (same state as the receive check).
On real hardware the controller must additionally be in Remote Control mode.

Usage (inside the dev container, URSim running):
    python scripts/smoke_test_move.py
"""

from __future__ import annotations

import math
import time

from ur5e.connection import RobotConnection
from ur5e.config import resolve_host

# --- smoke-test parameters (sim only; NOT a safety model) ---
JOINT_INDEX = 4                  # wrist 2 — small, low-consequence motion in sim
DELTA_RAD = math.radians(10.0)   # ~10 degrees
SPEED = 0.2                      # rad/s   (gentle)
ACCEL = 0.3                      # rad/s^2 (gentle)
TOL_RAD = math.radians(0.5)      # arrival tolerance


def close(a, b, tol=TOL_RAD):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def main() -> None:
    host = resolve_host("sim")
    print(f"connecting (control) to {host} ...")
    with RobotConnection(host, control=True) as r:
        if not r.connected:
            raise SystemExit("control interface not connected; is URSim ON/STARTED?")

        q0 = r.receive.getActualQ()
        print("start q (rad): ", [round(x, 4) for x in q0])

        target = list(q0)
        target[JOINT_INDEX] += DELTA_RAD
        print(f"moving joint {JOINT_INDEX} by {math.degrees(DELTA_RAD):.1f} deg ...")
        r.control.moveJ(target, SPEED, ACCEL)
        time.sleep(0.1)

        q1 = r.receive.getActualQ()
        print("reached q (rad):", [round(x, 4) for x in q1])
        reached = close(q1, target)
        print("  -> reached target:", reached)

        print("returning to start ...")
        r.control.moveJ(q0, SPEED, ACCEL)
        q2 = r.receive.getActualQ()
        back = close(q2, q0)
        print("  -> back at start:", back)

    print("disconnected cleanly.")
    print("SMOKE TEST:", "PASS" if (reached and back) else "FAIL")


if __name__ == "__main__":
    main()
