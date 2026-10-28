This repository is a UR5e cobot student framework. Follow
`context/INSTRUCTIONS.md` and use the facts in `context/KNOWLEDGE.md`:
write Python with `from ur5e import Robot, Workspace`, positions as
fractions of the taught workspace (`ws.pose(u, v, w)`, `ws.grid`, `ws.line`),
metres and radians only, `target="sim"` unless the student is in the lab,
never work around `SafetyViolation`, never invent methods not listed in
`context/KNOWLEDGE.md` §4, start new scripts from
`examples/function_templates/`, keep them in `students/<name>/`, and do not
edit `src/`, `scripts/`, `examples/` or `docs/`.
