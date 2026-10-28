"""Named connection targets.

The ONLY place the sim<->real host differs. Everything else in the framework
is host-agnostic.
"""

from __future__ import annotations

import os

# Env-overridable so the same code works from the host, from the dev container
# (URSIM_HOST=ursim via compose), and against the real arm (UR_HOST=<lan ip>).
HOSTS = {
    "sim": os.environ.get("URSIM_HOST", "127.0.0.1"),   # URSim
    "real": os.environ.get("UR_HOST", "0.0.0.0"),       # UR5e LAN IP
}


def resolve_host(target: str) -> str:
    try:
        return HOSTS[target]
    except KeyError:
        raise KeyError(f"unknown target {target!r}; known: {sorted(HOSTS)}") from None


# Hosts treated as simulation (no real-arm gate). Includes the resolved sim host.
_SIM_HOSTS = {"127.0.0.1", "localhost", "ursim"}


def is_sim(host: str) -> bool:
    return host in _SIM_HOSTS or host == HOSTS["sim"]
