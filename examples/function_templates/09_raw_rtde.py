"""09 — Advanced: the raw ur_rtde interfaces underneath Robot. SIM ONLY here.

Robot wraps three ur_rtde objects; they are yours when you need something
Robot does not offer (full API: ur_rtde docs, "RTDE*Interface"):

    robot.conn.receive   RTDEReceiveInterface   getActualQ(), getActualTCPPose(),
                                                getActualTCPForce(), getRobotMode(), ...
    robot.conn.control   RTDEControlInterface   moveJ(), moveL(), servoJ(), speedL(),
                                                getInverseKinematics(), ...
    robot.conn.io        RTDEIOInterface        setStandardDigitalOut(), ...

Raw control calls SKIP the framework's checks (workspace box, speed
ceilings). The arm does exactly what you send. That is why this template
is pinned to the simulator — on the real robot, stay with the Robot methods
unless you know exactly what you are doing and someone is at the e-stop.

What this does: read a few raw values, then a raw moveJ of wrist 3 by 10°
and back.

Usage:  python examples/function_templates/09_raw_rtde.py
"""

import math

from ur5e import Robot, Workspace

TARGET = "sim"     # keep it that way for raw calls


def main() -> None:
    if TARGET != "sim":
        raise SystemExit("this template bypasses the safety checks — sim only")
    ws = Workspace.load()
    with Robot(target=TARGET, profile=ws) as robot:
        rcv, ctl = robot.conn.receive, robot.conn.control
        print("robot mode      :", rcv.getRobotMode(), "(7 = running)")
        print("TCP force [N,Nm]:", [round(v, 1) for v in rcv.getActualTCPForce()])
        print("joint temps [C] :", [round(v, 1) for v in rcv.getJointTemperatures()])
        print("speed slider    :", rcv.getSpeedScaling())

        q = list(rcv.getActualQ())
        q[5] += math.radians(10)
        print("raw moveJ wrist 3 +10 deg (speed 0.5 rad/s, accel 0.5 rad/s^2) ...")
        ctl.moveJ(q, 0.5, 0.5)                 # (q, speed, acceleration) — no checks!
        q[5] -= math.radians(10)
        ctl.moveJ(q, 0.5, 0.5)
    print("done.")


if __name__ == "__main__":
    main()
