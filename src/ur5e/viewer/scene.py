"""Scene description + persistence for the viewer.

STATUS: HS26 student framework (sim-validated). Visualization only.

A *scene* is a self-contained folder::

    scenes/<name>/
        scene.json        <- this module reads/writes it
        models/*.stl|obj|glb|ply   <- student meshes, referenced relatively

so a scene can be zipped, committed or AirDropped as one unit.

File format ``ur5e-scene/1`` (JSON). All lengths in **metres**, all angles
in **degrees** unless the key says otherwise (``rotvec_rad``). Poses are
``{"xyz": [...], "rpy_deg": [...]}`` with RPY = PolyScope's RPY convention
(R = Rz(yaw) Ry(pitch) Rx(roll)). Everything is expressed in the robot's
**Base** frame (= PolyScope "Base" feature) unless stated.

The TCP is stored in UR's own representation (``xyz`` m + ``rotvec_rad``)
because that is what ``Robot.set_tool(..., tcp=...)`` / ``setTcp`` take and
what PolyScope Installation > TCP shows by default (mm and rad).

Nothing here is enforced on the robot. Safety planes in a scene are the
student's *intent* for the pendant; PolyScope remains authoritative.
"""

from __future__ import annotations

import json
import os
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np

FORMAT = "ur5e-scene/1"
SCENE_FILE = "scene.json"
MODELS_DIR = "models"
MESH_EXTS = (".stl", ".obj", ".glb", ".gltf", ".ply")
UNITS = ("mm", "m")
PLANE_MODES = ("normal", "reduced", "normal_and_reduced", "trigger_reduced", "disabled")
PLANE_MODE_LABELS = {          # exactly the words on the pendant
    "normal": "Normal",
    "reduced": "Reduced",
    "normal_and_reduced": "Normal & Reduced",
    "trigger_reduced": "Trigger Reduced Mode",
    "disabled": "Disabled",
}
FEATURE_BASE = "base"
FEATURE_TOOL = "tool"


# ----------------------------------------------------------------- data model
@dataclass
class Pose:
    xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def copy(self) -> "Pose":
        return Pose(list(self.xyz), list(self.rpy_deg))


@dataclass
class SceneObject:
    id: str
    mesh: str                      # relative to the scene folder
    mesh_units: str = "mm"
    pose: Pose = field(default_factory=Pose)
    color: str = "#9a9a9a"
    opacity: float = 1.0
    visible: bool = True


@dataclass
class Toolhead:
    mesh: Optional[str] = None     # relative to scene folder, None = no mesh
    mesh_units: str = "mm"
    mesh_offset: Pose = field(default_factory=Pose)   # mesh frame in tool0
    tcp_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])        # m, in tool0
    tcp_rotvec_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    payload_kg: float = 0.0        # documented only; the viewer does nothing with it
    color: str = "#d0d0d0"
    library: Optional[str] = None  # id of the toolheads/<id>/ preset it came from (informational)


@dataclass
class Feature:
    """A PolyScope-style *plane feature*: a named frame in Base. Safety planes
    copy a feature and use its Z axis as the plane normal."""
    id: str
    pose: Pose = field(default_factory=Pose)


@dataclass
class SafetyPlane:
    name: str
    feature: str = FEATURE_BASE    # "base" or a Feature.id ("tool" not allowed by PolyScope for planes)
    displacement: float = 0.0      # m along the feature Z axis (pendant shows mm)
    mode: str = "normal"
    restrict_elbow: bool = False
    size: List[float] = field(default_factory=lambda: [1.5, 1.5])   # render only
    visible: bool = True


@dataclass
class Scene:
    name: str = "default"
    description: str = ""
    robot_mount: Pose = field(default_factory=Pose)   # for later wall/truss mounts; identity now
    toolhead: Toolhead = field(default_factory=Toolhead)
    objects: List[SceneObject] = field(default_factory=list)
    features: List[Feature] = field(default_factory=list)
    safety_planes: List[SafetyPlane] = field(default_factory=list)
    view: dict = field(default_factory=dict)

    # --- lookups
    def object(self, oid: str) -> Optional[SceneObject]:
        return next((o for o in self.objects if o.id == oid), None)

    def feature(self, fid: str) -> Optional[Feature]:
        return next((f for f in self.features if f.id == fid), None)

    def plane(self, name: str) -> Optional[SafetyPlane]:
        return next((p for p in self.safety_planes if p.name == name), None)

    def unique_id(self, base: str, existing) -> str:
        base = slug(base) or "item"
        if base not in existing:
            return base
        i = 2
        while f"{base}_{i}" in existing:
            i += 1
        return f"{base}_{i}"


# ------------------------------------------------------------- (de)serialize
def _pose_to_json(p: Pose) -> dict:
    return {"xyz": [round(float(v), 6) for v in p.xyz],
            "rpy_deg": [round(float(v), 4) for v in p.rpy_deg]}


def _pose_from_json(d, what: str) -> Pose:
    if d is None:
        return Pose()
    if not isinstance(d, dict) or "xyz" not in d:
        raise SceneError(f"{what}: pose must be {{'xyz': [..], 'rpy_deg': [..]}}")
    return Pose([float(v) for v in d["xyz"]], [float(v) for v in d.get("rpy_deg", [0, 0, 0])])


class SceneError(ValueError):
    pass


def scene_to_json(s: Scene) -> dict:
    return {
        "format": FORMAT,
        "name": s.name,
        "description": s.description,
        "units_note": "lengths in metres, angles in degrees unless the key says _rad; frames in robot Base",
        "robot": {"model": "ur5e", "mount": _pose_to_json(s.robot_mount)},
        "toolhead": {
            "mesh": s.toolhead.mesh,
            "mesh_units": s.toolhead.mesh_units,
            "mesh_offset": _pose_to_json(s.toolhead.mesh_offset),
            "tcp": {"xyz": [round(float(v), 6) for v in s.toolhead.tcp_xyz],
                    "rotvec_rad": [round(float(v), 6) for v in s.toolhead.tcp_rotvec_rad]},
            "payload_kg": s.toolhead.payload_kg,
            "color": s.toolhead.color,
            "library": s.toolhead.library,
        },
        "objects": [{
            "id": o.id, "mesh": o.mesh, "mesh_units": o.mesh_units,
            "pose": _pose_to_json(o.pose), "color": o.color,
            "opacity": o.opacity, "visible": o.visible,
        } for o in s.objects],
        "features": [{"id": f.id, "type": "plane", "pose": _pose_to_json(f.pose)} for f in s.features],
        "safety_planes": [{
            "name": p.name, "feature": p.feature,
            "displacement": round(float(p.displacement), 6),
            "mode": p.mode, "restrict_elbow": p.restrict_elbow,
            "size": p.size, "visible": p.visible,
        } for p in s.safety_planes],
        "view": s.view,
    }


def scene_from_json(d: dict) -> Scene:
    if not isinstance(d, dict):
        raise SceneError("scene.json must contain a JSON object")
    fmt = d.get("format")
    if fmt != FORMAT:
        raise SceneError(f"unsupported scene format {fmt!r} (expected {FORMAT!r})")
    s = Scene(name=str(d.get("name", "scene")), description=str(d.get("description", "")))
    s.robot_mount = _pose_from_json((d.get("robot") or {}).get("mount"), "robot.mount")

    th = d.get("toolhead") or {}
    tcp = th.get("tcp") or {}
    s.toolhead = Toolhead(
        mesh=th.get("mesh") or None,
        mesh_units=_units(th.get("mesh_units", "mm"), "toolhead"),
        mesh_offset=_pose_from_json(th.get("mesh_offset"), "toolhead.mesh_offset"),
        tcp_xyz=[float(v) for v in tcp.get("xyz", [0, 0, 0])],
        tcp_rotvec_rad=[float(v) for v in tcp.get("rotvec_rad", [0, 0, 0])],
        payload_kg=float(th.get("payload_kg", 0.0)),
        color=str(th.get("color", "#d0d0d0")),
        library=th.get("library") or None,
    )
    for o in d.get("objects", []):
        if "id" not in o or "mesh" not in o:
            raise SceneError("each object needs 'id' and 'mesh'")
        s.objects.append(SceneObject(
            id=str(o["id"]), mesh=str(o["mesh"]),
            mesh_units=_units(o.get("mesh_units", "mm"), f"object {o['id']}"),
            pose=_pose_from_json(o.get("pose"), f"object {o['id']}"),
            color=str(o.get("color", "#9a9a9a")), opacity=float(o.get("opacity", 1.0)),
            visible=bool(o.get("visible", True))))
    for f in d.get("features", []):
        s.features.append(Feature(id=str(f["id"]), pose=_pose_from_json(f.get("pose"), f"feature {f['id']}")))
    for p in d.get("safety_planes", []):
        mode = str(p.get("mode", "normal"))
        if mode not in PLANE_MODES:
            raise SceneError(f"safety plane {p.get('name')!r}: mode must be one of {PLANE_MODES}")
        feat = str(p.get("feature", FEATURE_BASE))
        if feat != FEATURE_BASE and s.feature(feat) is None:
            raise SceneError(f"safety plane {p.get('name')!r} references unknown feature {feat!r}")
        s.safety_planes.append(SafetyPlane(
            name=str(p.get("name", "plane")), feature=feat,
            displacement=float(p.get("displacement", 0.0)), mode=mode,
            restrict_elbow=bool(p.get("restrict_elbow", False)),
            size=[float(v) for v in p.get("size", [1.5, 1.5])],
            visible=bool(p.get("visible", True))))
    s.view = dict(d.get("view") or {})
    return s


def _units(u, what) -> str:
    if u not in UNITS:
        raise SceneError(f"{what}: mesh_units must be 'mm' or 'm', got {u!r}")
    return u


# ------------------------------------------------------------------ storage
def scenes_root() -> Path:
    """``$UR5E_SCENES_DIR`` or ``./scenes`` (= /workspace/scenes in the container)."""
    return Path(os.environ.get("UR5E_SCENES_DIR", "scenes")).resolve()


def slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_\-]+", "_", name.strip()).strip("_-")


def scene_dir(name: str) -> Path:
    """Folder of a scene: an existing folder with exactly this name wins,
    otherwise the slugged name (used when creating)."""
    exact = scenes_root() / name
    if name and "/" not in name and exact.is_dir():
        return exact
    return scenes_root() / slug(name)


def list_scenes() -> List[str]:
    root = scenes_root()
    if not root.is_dir():
        return []
    return sorted(p.name for p in root.iterdir() if (p / SCENE_FILE).is_file())


def load_scene(name: str) -> Scene:
    path = scene_dir(name) / SCENE_FILE
    if not path.is_file():
        raise SceneError(f"no scene named {name!r} (expected {path})")
    with open(path) as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as e:
            raise SceneError(f"{path}: invalid JSON: {e}") from e
    s = scene_from_json(data)
    s.name = path.parent.name
    return s


def save_scene(s: Scene, name: Optional[str] = None) -> Path:
    name = name or s.name
    d = scene_dir(name)
    name = d.name
    (d / MODELS_DIR).mkdir(parents=True, exist_ok=True)
    s.name = name
    path = d / SCENE_FILE
    tmp = path.with_suffix(".json.tmp")
    with open(tmp, "w") as f:
        json.dump(scene_to_json(s), f, indent=2)
        f.write("\n")
    os.replace(tmp, path)
    return path


def copy_scene_models(src_name: str, dst_name: str) -> None:
    """Save-as helper: copy models/ so the new scene folder is self-contained."""
    src, dst = scene_dir(src_name) / MODELS_DIR, scene_dir(dst_name) / MODELS_DIR
    if src.is_dir() and src.resolve() != dst.resolve():
        dst.mkdir(parents=True, exist_ok=True)
        for p in src.iterdir():
            if p.is_file() and not (dst / p.name).exists():
                shutil.copy2(p, dst / p.name)


def new_scene(name: str) -> Scene:
    s = Scene(name=slug(name), description="")
    save_scene(s)
    return s


def list_models(name: str) -> List[str]:
    d = scene_dir(name) / MODELS_DIR
    if not d.is_dir():
        return []
    return sorted(f"{MODELS_DIR}/{p.name}" for p in d.iterdir()
                  if p.is_file() and p.suffix.lower() in MESH_EXTS)


def add_model_file(name: str, filename: str, content: bytes) -> str:
    """Store an uploaded mesh in the scene's models/ folder; returns relative path."""
    base = Path(filename).name
    if Path(base).suffix.lower() not in MESH_EXTS:
        raise SceneError(f"unsupported mesh type {base!r} (use {', '.join(MESH_EXTS)})")
    safe = slug(Path(base).stem) + Path(base).suffix.lower()
    d = scene_dir(name) / MODELS_DIR
    d.mkdir(parents=True, exist_ok=True)
    (d / safe).write_bytes(content)
    return f"{MODELS_DIR}/{safe}"


# ----------------------------------------------------------- toolhead library
TOOLHEAD_FORMAT = "ur5e-toolhead/1"
TOOLHEAD_FILE = "toolhead.json"


def toolheads_root() -> Path:
    """``$UR5E_TOOLHEADS_DIR`` or ``./toolheads`` — presets shared by all scenes:
    toolheads/<id>/toolhead.json + the mesh file next to it."""
    return Path(os.environ.get("UR5E_TOOLHEADS_DIR", "toolheads")).resolve()


def list_toolheads() -> List[str]:
    root = toolheads_root()
    if not root.is_dir():
        return []
    return sorted(p.name for p in root.iterdir() if (p / TOOLHEAD_FILE).is_file())


def _toolhead_to_json(th: Toolhead, mesh_name: Optional[str], name: str, notes: str = "") -> dict:
    return {
        "format": TOOLHEAD_FORMAT,
        "name": name,
        "notes": notes,
        "mesh": mesh_name,
        "mesh_units": th.mesh_units,
        "mesh_offset": _pose_to_json(th.mesh_offset),
        "tcp": {"xyz": [round(float(v), 6) for v in th.tcp_xyz],
                "rotvec_rad": [round(float(v), 6) for v in th.tcp_rotvec_rad]},
        "payload_kg": th.payload_kg,
        "color": th.color,
    }


def load_toolhead(tid: str) -> tuple:
    """-> (Toolhead with mesh = bare file name, preset dir, display name, notes)."""
    d = toolheads_root() / tid
    path = d / TOOLHEAD_FILE
    if not path.is_file():
        raise SceneError(f"no toolhead preset {tid!r} (expected {path})")
    with open(path) as f:
        try:
            j = json.load(f)
        except json.JSONDecodeError as e:
            raise SceneError(f"{path}: invalid JSON: {e}") from e
    if j.get("format") != TOOLHEAD_FORMAT:
        raise SceneError(f"{path}: unsupported format {j.get('format')!r}")
    tcp = j.get("tcp") or {}
    th = Toolhead(
        mesh=j.get("mesh") or None,
        mesh_units=_units(j.get("mesh_units", "mm"), f"toolhead {tid}"),
        mesh_offset=_pose_from_json(j.get("mesh_offset"), f"toolhead {tid} mesh_offset"),
        tcp_xyz=[float(v) for v in tcp.get("xyz", [0, 0, 0])],
        tcp_rotvec_rad=[float(v) for v in tcp.get("rotvec_rad", [0, 0, 0])],
        payload_kg=float(j.get("payload_kg", 0.0)),
        color=str(j.get("color", "#d0d0d0")),
        library=tid,
    )
    if th.mesh and not (d / th.mesh).is_file():
        raise SceneError(f"toolhead {tid!r}: mesh file {th.mesh!r} missing in {d}")
    return th, d, str(j.get("name", tid)), str(j.get("notes", ""))


def apply_toolhead_preset(scene: Scene, tid: str) -> Toolhead:
    """Copy a library preset into the scene: mesh goes to scenes/<name>/models/
    (so the scene folder stays self-contained), fields are replaced."""
    th, d, _name, _notes = load_toolhead(tid)
    if th.mesh:
        dst_dir = scene_dir(scene.name) / MODELS_DIR
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(d / th.mesh, dst_dir / th.mesh)
        th.mesh = f"{MODELS_DIR}/{th.mesh}"
    scene.toolhead = th
    return th


def save_toolhead_preset(scene: Scene, tid: str, name: Optional[str] = None, notes: str = "") -> Path:
    """Reverse: store the scene's current toolhead as toolheads/<tid>/."""
    tid = slug(tid)
    if not tid:
        raise SceneError("toolhead preset needs a name")
    th = scene.toolhead
    d = toolheads_root() / tid
    d.mkdir(parents=True, exist_ok=True)
    mesh_name = None
    if th.mesh:
        src = resolve_mesh_path(scene.name, th.mesh)
        if not src.is_file():
            raise SceneError(f"toolhead mesh not found: {src}")
        mesh_name = src.name
        if src.resolve() != (d / mesh_name).resolve():
            shutil.copy2(src, d / mesh_name)
    path = d / TOOLHEAD_FILE
    with open(path, "w") as f:
        json.dump(_toolhead_to_json(th, mesh_name, name or tid, notes), f, indent=2)
        f.write("\n")
    th.library = tid
    return path


def resolve_mesh_path(scene_name: str, rel: str) -> Path:
    p = Path(rel)
    return p if p.is_absolute() else scene_dir(scene_name) / p


def load_mesh(scene_name: str, rel: str, units: str):
    """Load a student mesh as a single trimesh.Trimesh in metres."""
    import trimesh  # local import keeps scene.py importable without it
    path = resolve_mesh_path(scene_name, rel)
    if not path.is_file():
        raise SceneError(f"mesh file not found: {path}")
    m = trimesh.load(str(path), force="mesh")
    if not isinstance(m, trimesh.Trimesh) or m.is_empty:
        raise SceneError(f"could not read a mesh from {path}")
    if units == "mm":
        m = m.copy()
        m.apply_scale(0.001)
    return m


def mesh_extent_mm(mesh) -> np.ndarray:
    return np.asarray(mesh.extents, dtype=float) * 1000.0
