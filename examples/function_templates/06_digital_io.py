"""06 — Digital I/O: read inputs, switch outputs. No motion.

    robot.read_digital_in(pin)         -> True/False   standard inputs 0..7 (controller)
    robot.set_digital_out(pin, value)                  standard outputs 0..7
    robot.set_tool_digital_out(pin, value)             the two outputs on the tool flange (0, 1)

Outputs show up live in the pendant's I/O tab (localhost:6080 → I/O): watch
Digital Output 0 tick on and off while this runs. Inputs in the SIMULATOR
always read 0 — there is nothing wired to them and the sim cannot fake them
for us. On the real robot a button/sensor between 24V and DI0 reads True.

What this does: blink DO0 three times, pulse the tool output once, print
all eight inputs.

Usage:  python examples/function_templates/06_digital_io.py
"""

import time

from ur5e import Robot, Workspace

TARGET = "sim"


def main() -> None:
    ws = Workspace.load()
    with Robot(target=TARGET, profile=ws) as robot:
        print("inputs DI0..7:", [robot.read_digital_in(i) for i in range(8)])

        for k in range(3):
            robot.set_digital_out(0, True)
            print(f"DO0 on  ({k + 1}/3)  <- look at the I/O tab")
            time.sleep(0.7)
            robot.set_digital_out(0, False)
            print("DO0 off")
            time.sleep(0.7)

        robot.set_tool_digital_out(0, True)
        time.sleep(0.5)
        robot.set_tool_digital_out(0, False)
        print("tool output 0 pulsed")

        # a typical sensor-gated pattern (always 'no' in the sim):
        if robot.read_digital_in(0):
            print("DI0 is HIGH -> would run the process now")
        else:
            print("DI0 is LOW  -> skipping (in the sim this is always the case)")
    print("done.")


if __name__ == "__main__":
    main()
