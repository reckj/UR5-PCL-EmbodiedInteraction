"""Live RTDE 3D viewer + scene editor for the UR5e — SIM or REAL.

STATUS: HS26 student framework — sim-validated (URSim round-trip 2026-10-28). Display only.

Renders the arm (real UR5e link meshes) in a browser from live RTDE joint
state (getActualQ), so you can SEE motion even in Remote Control mode (where
PolyScope hides the 3D view), and later against the real arm. Receive-only:
it never commands motion; run it next to a control script.

Around the arm you can place your own 3D models (table, fixtures, work
pieces), attach a toolhead with a PolyScope-style TCP offset, and draw safety
planes in the same terms as Installation > Safety > Planes. All of that is
saved as a *scene*: scenes/<name>/scene.json + models/ next to it.
NOTHING drawn here limits the robot — it is a planning aid only.

Open in the Mac browser:  http://localhost:7001/

Usage (inside the dev container, URSim running):
    python scripts/rtde_viewer.py --target sim                 # scene 'default'
    python scripts/rtde_viewer.py --target sim --scene my_table
    python scripts/rtde_viewer.py --no-robot --scene my_table   # edit a scene offline,
                                                                # pose the arm with sliders
The implementation lives in ur5e.viewer (server.py, scene.py, ...).
"""

from __future__ import annotations

import argparse
import os

from ur5e.config import resolve_host
from ur5e.viewer.server import run_viewer


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g_ = ap.add_mutually_exclusive_group(required=True)
    g_.add_argument("--target", choices=["sim", "real"])
    g_.add_argument("--host", help="robot/URSim IP")
    g_.add_argument("--no-robot", action="store_true", help="offline: no RTDE, manual joint sliders")
    ap.add_argument("--scene", default=os.environ.get("VIEWER_SCENE", "default"),
                    help="scene name = folder under scenes/ (created if missing)")
    ap.add_argument("--rate", type=float, default=20.0, help="update Hz")
    ap.add_argument("--port", type=int, default=7000, help="container port (compose maps 7001->7000)")
    ap.add_argument("--joints", help="offline only: start pose, 6 joint angles in deg, e.g. 0,-90,90,-90,-90,0")
    args = ap.parse_args()

    host = None if args.no_robot else (args.host or resolve_host(args.target))
    q0 = None
    if args.joints:
        q0 = [float(v) for v in args.joints.split(",")]
        if len(q0) != 6:
            ap.error("--joints needs 6 comma-separated values (deg)")
    run_viewer(host=host, scene_name=args.scene, rate=args.rate, port=args.port, joints_deg=q0)


if __name__ == "__main__":
    main()
