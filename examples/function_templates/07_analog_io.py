"""07 — Analog I/O: read voltages, set an output voltage. No motion.

    robot.read_analog_in(pin)        -> float volts  (standard analog inputs 0 and 1)
    robot.set_analog_out(pin, volts)                 standard analog outputs 0 and 1

Analog inputs are for sensors with a voltage output (distance, force,
potentiometer, 0–10 V). In the simulator they read 0.0 V. Outputs are
visible in the pendant's I/O tab (set its mode to Voltage there).

What this does: print both inputs 10x (half a second apart), ramp AO0 from
0 to 5 V and back to 0.

Usage:  python examples/function_templates/07_analog_io.py
"""

import time

from ur5e import Robot, Workspace

TARGET = "sim"
THRESHOLD_V = 2.5


def main() -> None:
    ws = Workspace.load()
    with Robot(target=TARGET, profile=ws) as robot:
        for _ in range(10):
            a0, a1 = robot.read_analog_in(0), robot.read_analog_in(1)
            state = "above" if a0 > THRESHOLD_V else "below"
            print(f"AI0 = {a0:5.2f} V ({state} {THRESHOLD_V} V)   AI1 = {a1:5.2f} V")
            time.sleep(0.5)

        print("ramping AO0 0 -> 5 V -> 0")
        for v in (1.0, 2.0, 3.0, 4.0, 5.0, 0.0):
            robot.set_analog_out(0, v)
            time.sleep(0.4)
    print("done.")


if __name__ == "__main__":
    main()
