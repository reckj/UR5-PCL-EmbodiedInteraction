"""Verify RTDE connectivity. Receive-only: uploads no script, moves nothing.

Usage:
    python scripts/check_connection.py --target sim
    python scripts/check_connection.py --host 192.168.1.10
"""

from __future__ import annotations

import argparse

from ur5e.connection import RobotConnection
from ur5e.config import resolve_host


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--target", choices=["sim", "real"])
    g.add_argument("--host")
    args = ap.parse_args()

    host = args.host or resolve_host(args.target)
    print(f"Connecting (receive-only) to {host} ...")
    with RobotConnection(host) as r:
        print("connected   :", r.connected)
        print("robot mode  :", r.receive.getRobotMode())
        print("actual q    :", [round(x, 4) for x in r.receive.getActualQ()])
        print("tcp pose    :", [round(x, 4) for x in r.receive.getActualTCPPose()])
    print("disconnected cleanly.")


if __name__ == "__main__":
    main()
