# toolheads/ — toolhead presets for the viewer

One folder per toolhead, shared by all scenes:

    toolheads/<id>/toolhead.json   mesh file name, units, mesh offset in tool0, TCP (m + rotvec rad), payload
    toolheads/<id>/<mesh>.stl      the model, next to the json

In the viewer, *Toolhead + TCP → Toolhead preset → Use selected preset* copies the
mesh into the current scene's `models/` and fills the TCP/offset fields; *Save
current toolhead to library* does the reverse. The TCP in a preset is a starting
point — verify it against the real tool and the pendant (Installation → TCP).
Nothing here is enforced on the robot.
