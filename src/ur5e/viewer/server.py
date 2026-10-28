"""Live UR5e viewer + scene editor (viser) — SIM or REAL, receive-only.

STATUS: HS26 student framework — sim-validated (URSim round-trip 2026-10-28). Display only.

What it does
- renders the UR5e with real link meshes from live RTDE joint state
  (``getActualQ``), exactly like the previous meshcat viewer, but through
  viser (one HTTP+WebSocket server, browser at http://localhost:7001/)
- lets students place their own STL/OBJ/GLB models (table, fixtures,
  work pieces) with a gizmo + numeric fields, attach a toolhead mesh with a
  PolyScope-style TCP offset, and define safety planes in PolyScope terms
- saves/loads all of that as ``scenes/<name>/scene.json`` (see scene.py)

What it does NOT do
- command motion, block motion, or enforce anything. Safety planes here are
  drawn, never applied. PolyScope's Safety Configuration is authoritative.

Threads: an RTDE receive thread (reconnects forever), viser's own callback
threads for GUI events, and the main thread which pushes transforms at
``rate`` Hz. ``self.lock`` guards the shared joint/TCP state; scene edits are
serialized through ``self.scene_lock``.
"""

from __future__ import annotations

import math
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import trimesh
import viser

from . import robot_model as rm
from . import safety_planes as sp
from . import scene as sc
from .transforms import (quat_wxyz_to_rot, rot_to_rotvec, rot_to_rpy, rotvec_to_rot,
                         rpy_to_rot, tf_from_xyz_rpy_deg, tf_to_viser, make_tf)

BANNER = (
    "**UR5e viewer.** Visualization only: nothing "
    "drawn here limits the robot. Re-enter safety planes and the TCP on the "
    "pendant yourself; PolyScope stays authoritative."
)


def _hex_to_rgb(h: str):
    h = h.lstrip("#")
    if len(h) != 6:
        return (154, 154, 154)
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def _rgb_to_hex(rgb) -> str:
    return "#%02x%02x%02x" % tuple(int(v) for v in rgb)


def _fmt_mm(v) -> str:
    return " ".join(f"{1000 * float(x):7.1f}" for x in v)


class ViewerApp:
    def __init__(self, server: viser.ViserServer, scene_name: str, *, offline: bool = False):
        self.server = server
        self.offline = offline
        self.lock = threading.Lock()
        self.scene_lock = threading.RLock()
        self.q = np.array(rm.HOME_Q)
        self.controller_tcp: Optional[np.ndarray] = None   # getActualTCPPose, UR pose
        self.last_rx = 0.0
        self.robot_ts = None
        self.rtde_status = "offline (--no-robot)" if offline else "RTDE: starting …"
        self._stop = False
        self._syncing = False         # guard against gizmo <-> field feedback loops

        self.scene: sc.Scene = self._load_or_create(scene_name)
        self._obj_nodes: Dict[str, dict] = {}
        self._plane_nodes: Dict[str, dict] = {}
        self._feat_nodes: Dict[str, dict] = {}
        self._dyn_folders: list = []
        self._plane_status_text: Dict[str, viser.GuiTextHandle] = {}
        self._last_plane_colors: Dict[str, tuple] = {}

        server.scene.set_up_direction("+z")
        server.gui.set_panel_label("UR5e viewer")
        server.gui.configure_theme(control_layout="collapsible", control_width="medium",
                                   show_share_button=False, brand_color=(40, 120, 200))
        self._build_world()
        self._build_robot()
        self._build_static_gui()
        self._apply_scene_to_3d()      # 3D first: the GUI shows mesh sizes from it
        self._rebuild_scene_gui()
        self._apply_view()

    # ------------------------------------------------------------ scene I/O
    @staticmethod
    def _load_or_create(name: str) -> sc.Scene:
        try:
            return sc.load_scene(name)
        except sc.SceneError as e:
            if "no scene named" in str(e):
                print(f"[viewer] creating new scene {name!r} at {sc.scene_dir(name)}")
                return sc.new_scene(name)
            raise

    def _notify(self, msg: str, error: bool = False):
        print(("[viewer] ERROR " if error else "[viewer] ") + msg)
        try:
            self._msg.value = msg
        except Exception:
            pass

    # ------------------------------------------------------------ 3D: world
    def _build_world(self):
        s = self.server.scene
        self._grid = s.add_grid("/grid", width=3.0, height=3.0, plane="xy", cell_size=0.1,
                                section_size=1.0, cell_color=(205, 205, 205),
                                section_color=(120, 120, 120), section_thickness=1.5,
                                position=(0.0, 0.0, -0.0005))
        for d in (0.5, 1.0):
            s.add_label(f"/grid/lx{d}", f"x {d:.1f} m", position=(d, 0.02, 0.0), font_screen_scale=0.8)
            s.add_label(f"/grid/ly{d}", f"y {d:.1f} m", position=(0.02, d, 0.0), font_screen_scale=0.8)
        self._base_frame = s.add_frame("/frames/base", axes_length=0.15, axes_radius=0.004,
                                       origin_radius=0.008)
        s.add_label("/frames/base/label", "Base", position=(0.0, 0.0, 0.17), font_screen_scale=0.8)

    # ------------------------------------------------------------ 3D: robot
    def _build_robot(self):
        s = self.server.scene
        self._link_handles = {}
        for link in rm.LINKS:
            self._link_handles[link.name] = s.add_glb(f"/robot/{link.name}", rm.mesh_bytes(link))
        # tool0 node: everything tool-attached hangs under it (relative transforms)
        self._tool0 = s.add_frame("/tool0", show_axes=False)
        self._tool0_axes = s.add_frame("/tool0/axes", axes_length=0.06, axes_radius=0.003,
                                       origin_radius=0.005)
        self._tcp_frame = s.add_frame("/tool0/tcp", axes_length=0.08, axes_radius=0.0035,
                                      origin_radius=0.007, origin_color=(255, 60, 60))
        s.add_label("/tool0/tcp/label", "TCP", position=(0, 0, 0.09), font_screen_scale=0.8)
        self._elbow = s.add_icosphere("/elbow", radius=0.012, color=(255, 120, 0), visible=False)
        self._toolhead_mesh = None
        self._update_robot(rm.fk(self.q))

    def _update_robot(self, frames):
        poses = rm.link_world_poses(frames)
        for name, T in poses.items():
            w, p = tf_to_viser(T)
            h = self._link_handles[name]
            h.wxyz, h.position = w, p
        w, p = tf_to_viser(frames["tool0"])
        self._tool0.wxyz, self._tool0.position = w, p
        self._elbow.position = frames["elbow"][:3, 3]

    # ------------------------------------------------------------ 3D: scene
    def _apply_scene_to_3d(self):
        with self.scene_lock:
            for oid in list(self._obj_nodes):
                self._remove_obj_3d(oid)
            for o in self.scene.objects:
                self._add_obj_3d(o)
            for fid in list(self._feat_nodes):
                self._remove_feat_3d(fid)
            for f in self.scene.features:
                self._add_feat_3d(f)
            for name in list(self._plane_nodes):
                self._remove_plane_3d(name)
            for p in self.scene.safety_planes:
                self._add_plane_3d(p)
            self._apply_toolhead_3d()

    # objects
    def _add_obj_3d(self, o: sc.SceneObject):
        s = self.server.scene
        try:
            mesh = sc.load_mesh(self.scene.name, o.mesh, o.mesh_units)
        except Exception as e:
            self._notify(f"object {o.id!r}: {e}", error=True)
            self._obj_nodes[o.id] = {"mesh": None, "gizmo": None, "extent": None}
            return
        T = tf_from_xyz_rpy_deg(o.pose.xyz, o.pose.rpy_deg)
        w, p = tf_to_viser(T)
        h = s.add_mesh_simple(f"/objects/{o.id}", np.asarray(mesh.vertices, dtype=np.float32),
                              np.asarray(mesh.faces, dtype=np.uint32), color=_hex_to_rgb(o.color),
                              opacity=(None if o.opacity >= 0.999 else o.opacity), side="double",
                              wxyz=w, position=p, visible=o.visible)
        self._obj_nodes[o.id] = {"mesh": h, "gizmo": None, "extent": sc.mesh_extent_mm(mesh)}

    def _remove_obj_3d(self, oid: str):
        n = self._obj_nodes.pop(oid, None)
        if not n:
            return
        for k in ("mesh", "gizmo"):
            if n.get(k) is not None:
                n[k].remove()

    def _set_obj_pose_3d(self, o: sc.SceneObject, *, move_gizmo=True):
        n = self._obj_nodes.get(o.id)
        if not n:
            return
        w, p = tf_to_viser(tf_from_xyz_rpy_deg(o.pose.xyz, o.pose.rpy_deg))
        if n["mesh"] is not None:
            n["mesh"].wxyz, n["mesh"].position = w, p
        if move_gizmo and n["gizmo"] is not None:
            n["gizmo"].wxyz, n["gizmo"].position = w, p

    # features
    def _add_feat_3d(self, f: sc.Feature):
        s = self.server.scene
        w, p = tf_to_viser(tf_from_xyz_rpy_deg(f.pose.xyz, f.pose.rpy_deg))
        h = s.add_frame(f"/features/{f.id}", axes_length=0.12, axes_radius=0.003,
                        origin_radius=0.006, origin_color=(80, 200, 120), wxyz=w, position=p)
        s.add_label(f"/features/{f.id}/label", f"feature: {f.id}", position=(0, 0, 0.13),
                    font_screen_scale=0.8)
        self._feat_nodes[f.id] = {"frame": h, "gizmo": None}

    def _remove_feat_3d(self, fid: str):
        n = self._feat_nodes.pop(fid, None)
        if not n:
            return
        for k in ("frame", "gizmo"):
            if n.get(k) is not None:
                n[k].remove()

    def _set_feat_pose_3d(self, f: sc.Feature, *, move_gizmo=True):
        n = self._feat_nodes.get(f.id)
        if not n:
            return
        w, p = tf_to_viser(tf_from_xyz_rpy_deg(f.pose.xyz, f.pose.rpy_deg))
        n["frame"].wxyz, n["frame"].position = w, p
        if move_gizmo and n["gizmo"] is not None:
            n["gizmo"].wxyz, n["gizmo"].position = w, p
        # planes copied from this feature move with it
        for p_ in self.scene.safety_planes:
            if p_.feature == f.id:
                self._set_plane_pose_3d(p_)

    # planes
    @staticmethod
    def _arrow_mesh(length=0.18, radius=0.006):
        shaft = trimesh.creation.cylinder(radius=radius, height=length * 0.7, sections=16)
        shaft.apply_translation((0, 0, length * 0.35))
        head = trimesh.creation.cone(radius=radius * 3, height=length * 0.3, sections=16)
        head.apply_translation((0, 0, length * 0.7))
        return trimesh.util.concatenate([shaft, head])

    def _add_plane_3d(self, p: sc.SafetyPlane):
        s = self.server.scene
        try:
            T = sp.plane_tf(self.scene, p)
        except Exception as e:
            self._notify(f"plane {p.name!r}: {e}", error=True)
            return
        w, pos = tf_to_viser(T)
        col = sp.render_color(p, None)
        box = s.add_box(f"/planes/{p.name}", dimensions=(p.size[0], p.size[1], 0.002), color=col,
                        opacity=0.35, side="double", wxyz=w, position=pos, visible=p.visible)
        arrow = self._arrow_mesh()
        arr = s.add_mesh_simple(f"/planes/{p.name}/arrow", np.asarray(arrow.vertices, np.float32),
                                np.asarray(arrow.faces, np.uint32), color=col, flat_shading=True)
        txt = f"{p.name} — {sc.PLANE_MODE_LABELS[p.mode]}" + (" · elbow" if p.restrict_elbow else "")
        lab = s.add_label(f"/planes/{p.name}/label", txt, position=(0, 0, 0.2), font_screen_scale=0.8)
        self._plane_nodes[p.name] = {"box": box, "arrow": arr, "label": lab}
        self._last_plane_colors[p.name] = col

    def _remove_plane_3d(self, name: str):
        n = self._plane_nodes.pop(name, None)
        if n:
            n["box"].remove()   # children (arrow/label) go with it
        self._last_plane_colors.pop(name, None)

    def _set_plane_pose_3d(self, p: sc.SafetyPlane):
        n = self._plane_nodes.get(p.name)
        if not n:
            return
        try:
            w, pos = tf_to_viser(sp.plane_tf(self.scene, p))
        except Exception as e:
            self._notify(f"plane {p.name!r}: {e}", error=True)
            return
        n["box"].wxyz, n["box"].position = w, pos
        n["box"].visible = p.visible
        n["label"].text = f"{p.name} — {sc.PLANE_MODE_LABELS[p.mode]}" + (" · elbow" if p.restrict_elbow else "")

    def _refresh_plane_3d(self, p: sc.SafetyPlane):
        """Mode/size changed -> rebuild that plane's geometry."""
        self._remove_plane_3d(p.name)
        self._add_plane_3d(p)

    # toolhead
    def _apply_toolhead_3d(self):
        th = self.scene.toolhead
        if self._toolhead_mesh is not None:
            self._toolhead_mesh.remove()
            self._toolhead_mesh = None
        if th.mesh:
            try:
                mesh = sc.load_mesh(self.scene.name, th.mesh, th.mesh_units)
                w, p = tf_to_viser(tf_from_xyz_rpy_deg(th.mesh_offset.xyz, th.mesh_offset.rpy_deg))
                self._toolhead_mesh = self.server.scene.add_mesh_simple(
                    "/tool0/toolhead", np.asarray(mesh.vertices, np.float32),
                    np.asarray(mesh.faces, np.uint32), color=_hex_to_rgb(th.color),
                    side="double", wxyz=w, position=p)
            except Exception as e:
                self._notify(f"toolhead mesh: {e}", error=True)
        T_tcp = make_tf(rotvec_to_rot(th.tcp_rotvec_rad), th.tcp_xyz)
        w, p = tf_to_viser(T_tcp)
        self._tcp_frame.wxyz, self._tcp_frame.position = w, p

    def _apply_view(self, move_clients: bool = False):
        v = self.scene.view or {}
        pos = tuple(v.get("camera_position", (1.7, -1.7, 1.2)))
        tgt = tuple(v.get("camera_target", (0.2, 0.0, 0.3)))
        cam = self.server.initial_camera          # new browser tabs
        cam.position, cam.look_at = pos, tgt
        if move_clients:                          # tabs already open (scene load)
            for client in self.server.get_clients().values():
                try:
                    client.camera.position, client.camera.look_at = pos, tgt
                except Exception:
                    pass

    # ------------------------------------------------------------ GUI: static
    def _build_static_gui(self):
        g = self.server.gui
        g.add_markdown(BANNER)
        self._msg = g.add_text("Message", "", disabled=True)

        with g.add_folder("Robot"):
            self._st_rtde = g.add_text("Connection", self.rtde_status, disabled=True)
            self._st_q = g.add_text("Joints [deg]", "", disabled=True)
            self._st_tcp_ctrl = g.add_text("Controller TCP [mm | rotvec rad]", "", disabled=True,
                                           hint="getActualTCPPose(): the pendant's active TCP")
            self._st_tcp_view = g.add_text("Viewer TCP [mm | rotvec rad]", "", disabled=True,
                                           hint="FK(q) ∘ TCP from this scene")
            self._st_match = g.add_text("TCP check", "", disabled=True,
                                        hint="Scene TCP vs the TCP active on the controller")
            if self.offline:
                with g.add_folder("Manual joints [deg] (offline)", expand_by_default=False):
                    self._sliders = []
                    for i, n in enumerate(rm.JOINT_NAMES):
                        h = g.add_slider(n, min=-360, max=360, step=0.5,
                                         initial_value=round(math.degrees(rm.HOME_Q[i]), 1))
                        h.on_update(lambda _e, i=i, h=h: self._set_manual_joint(i, h.value))
                        self._sliders.append(h)

        with g.add_folder("Scene file"):
            self._sc_name = g.add_text("Current scene", self.scene.name, disabled=True)
            self._sc_dd = g.add_dropdown("Available", self._scene_options(), initial_value=self.scene.name)
            b_load = g.add_button("Load selected", icon=viser.Icon.FOLDER_OPEN)
            b_save = g.add_button("Save", icon=viser.Icon.DEVICE_FLOPPY, color="green")
            self._sc_new = g.add_text("New name", "")
            b_saveas = g.add_button("Save as new name", hint="Copies the models/ folder too")
            b_new = g.add_button("New empty scene with that name")
            b_reload = g.add_button("Reload from disk", hint="Discard unsaved changes")
            self._sc_descr = g.add_text("Description", self.scene.description)
            self._sc_descr.on_update(lambda _e: setattr(self.scene, "description", self._sc_descr.value))
            b_view = g.add_button("Store current camera in scene")
            b_load.on_click(lambda _e: self._gui_load(self._sc_dd.value))
            b_save.on_click(lambda _e: self._gui_save())
            b_saveas.on_click(lambda _e: self._gui_save_as(self._sc_new.value))
            b_new.on_click(lambda _e: self._gui_new(self._sc_new.value))
            b_reload.on_click(lambda _e: self._gui_load(self.scene.name))
            b_view.on_click(lambda e: self._gui_store_view(e.client))

        with g.add_folder("View", expand_by_default=False):
            c_grid = g.add_checkbox("Grid (0.1 m cells, 1 m lines)", True)
            c_base = g.add_checkbox("Base frame", True)
            c_t0 = g.add_checkbox("tool0 frame (flange)", True)
            c_tcp = g.add_checkbox("TCP frame", True)
            c_elb = g.add_checkbox("Elbow marker", False)
            c_grid.on_update(lambda _e: setattr(self._grid, "visible", c_grid.value))
            c_base.on_update(lambda _e: setattr(self._base_frame, "visible", c_base.value))
            c_t0.on_update(lambda _e: setattr(self._tool0_axes, "visible", c_t0.value))
            c_tcp.on_update(lambda _e: setattr(self._tcp_frame, "visible", c_tcp.value))
            c_elb.on_update(lambda _e: setattr(self._elbow, "visible", c_elb.value))

    def _scene_options(self):
        opts = sc.list_scenes()
        if self.scene.name not in opts:
            opts.append(self.scene.name)
        return opts

    def _set_manual_joint(self, i: int, deg: float):
        with self.lock:
            self.q = self.q.copy()
            self.q[i] = math.radians(deg)

    # scene-file actions
    def _gui_save(self):
        with self.scene_lock:
            try:
                path = sc.save_scene(self.scene)
                self._notify(f"saved {path}")
            except Exception as e:
                self._notify(f"save failed: {e}", error=True)
        self._sc_dd.options = self._scene_options()

    def _gui_save_as(self, name: str):
        name = sc.slug(name)
        if not name:
            return self._notify("enter a name first", error=True)
        with self.scene_lock:
            old = self.scene.name
            try:
                sc.copy_scene_models(old, name)
                sc.save_scene(self.scene, name)
            except Exception as e:
                return self._notify(f"save-as failed: {e}", error=True)
        self._sc_name.value = name
        self._sc_dd.options = self._scene_options()
        self._sc_dd.value = name
        self._notify(f"saved as scene {name!r}")

    def _gui_new(self, name: str):
        name = sc.slug(name)
        if not name:
            return self._notify("enter a name first", error=True)
        if name in sc.list_scenes():
            return self._notify(f"scene {name!r} exists — load it instead", error=True)
        sc.new_scene(name)
        self._gui_load(name)

    def _gui_load(self, name: str):
        with self.scene_lock:
            try:
                self.scene = sc.load_scene(name)
            except Exception as e:
                return self._notify(f"load failed: {e}", error=True)
            self._sc_name.value = self.scene.name
            self._sc_descr.value = self.scene.description
            self._sc_dd.options = self._scene_options()
            self._sc_dd.value = self.scene.name
            self._apply_scene_to_3d()
            self._rebuild_scene_gui()
            self._apply_view(move_clients=bool(self.scene.view))
        self._notify(f"loaded scene {name!r}")

    def _gui_store_view(self, client):
        if client is None:
            return
        self.scene.view = {"camera_position": [round(float(v), 3) for v in client.camera.position],
                           "camera_target": [round(float(v), 3) for v in client.camera.look_at]}
        self._notify("camera stored in scene (save to keep)")

    # ------------------------------------------------------------ GUI: dynamic
    def _rebuild_scene_gui(self):
        for f in self._dyn_folders:
            try:
                f.remove()
            except Exception:
                pass
        self._dyn_folders = []
        self._plane_status_text = {}
        g = self.server.gui
        with self.scene_lock:
            f_obj = g.add_folder("Objects")
            self._dyn_folders.append(f_obj)
            with f_obj:
                self._build_add_object_gui()
                for o in self.scene.objects:
                    self._build_object_gui(o)
            f_th = g.add_folder("Toolhead + TCP")
            self._dyn_folders.append(f_th)
            with f_th:
                self._build_toolhead_gui()
            f_feat = g.add_folder("Features (for planes)", expand_by_default=False)
            self._dyn_folders.append(f_feat)
            with f_feat:
                self._build_add_feature_gui()
                for f in self.scene.features:
                    self._build_feature_gui(f)
            f_pl = g.add_folder("Safety planes (display only)")
            self._dyn_folders.append(f_pl)
            with f_pl:
                self._build_add_plane_gui()
                for p in self.scene.safety_planes:
                    self._build_plane_gui(p)

    # -- objects
    def _build_add_object_gui(self):
        g = self.server.gui
        g.add_markdown("Models live in `scenes/<scene>/models/`. Upload or pick one already there.")
        units = g.add_dropdown("Units of the file", ["mm", "m"], initial_value="mm",
                               hint="Most CAD exports (Fusion, Rhino, Blender-mm) are mm")
        up = g.add_upload_button("Upload STL / OBJ / GLB", icon=viser.Icon.UPLOAD,
                                 mime_type=".stl,.obj,.glb,.gltf,.ply")
        existing = g.add_dropdown("Existing model file", [""] + sc.list_models(self.scene.name))
        b_add = g.add_button("Add existing as object")

        def on_upload(ev):
            f = ev.target.value
            try:
                rel = sc.add_model_file(self.scene.name, f.name, f.content)
            except Exception as e:
                return self._notify(f"upload failed: {e}", error=True)
            self._add_object(rel, units.value)
            existing.options = [""] + sc.list_models(self.scene.name)

        up.on_upload(on_upload)
        b_add.on_click(lambda _e: self._add_object(existing.value, units.value) if existing.value else None)

    def _add_object(self, rel: str, units: str):
        with self.scene_lock:
            oid = self.scene.unique_id(Path(rel).stem, {o.id for o in self.scene.objects})
            o = sc.SceneObject(id=oid, mesh=rel, mesh_units=units)
            self.scene.objects.append(o)
            self._add_obj_3d(o)
        ext = self._obj_nodes.get(oid, {}).get("extent")
        if ext is not None:
            self._notify(f"added {oid}: {ext[0]:.0f} × {ext[1]:.0f} × {ext[2]:.0f} mm "
                         f"(if that is 1000× off, change the units and re-add)")
        self._rebuild_scene_gui()

    def _build_object_gui(self, o: sc.SceneObject):
        g = self.server.gui
        ext = self._obj_nodes.get(o.id, {}).get("extent")
        size = f"{ext[0]:.0f}×{ext[1]:.0f}×{ext[2]:.0f} mm" if ext is not None else "mesh missing"
        with g.add_folder(f"{o.id}  ({size})", expand_by_default=False):
            g.add_markdown(f"`{o.mesh}` · {o.mesh_units}")
            vis = g.add_checkbox("Visible", o.visible)
            giz = g.add_checkbox("Gizmo (drag in 3D)", False)
            pos = g.add_vector3("Position [mm]", tuple(1000 * v for v in o.pose.xyz), step=1.0)
            rot = g.add_vector3("Rotation RPY [deg]", tuple(o.pose.rpy_deg), step=1.0)
            col = g.add_rgb("Colour", _hex_to_rgb(o.color))
            opa = g.add_slider("Opacity", 0.1, 1.0, 0.05, o.opacity)
            rm_ = g.add_button("Remove object", color="red")

            def from_fields(_e=None):
                if self._syncing:
                    return
                o.pose.xyz = [v / 1000.0 for v in pos.value]
                o.pose.rpy_deg = list(rot.value)
                self._set_obj_pose_3d(o)

            def on_gizmo(ev):
                tc = ev.target
                o.pose.xyz = [float(v) for v in tc.position]
                o.pose.rpy_deg = [float(v) for v in np.degrees(rot_to_rpy(quat_wxyz_to_rot(tc.wxyz)))]
                self._set_obj_pose_3d(o, move_gizmo=False)
                self._syncing = True
                try:
                    pos.value = tuple(1000 * v for v in o.pose.xyz)
                    rot.value = tuple(round(v, 2) for v in o.pose.rpy_deg)
                finally:
                    self._syncing = False

            def toggle_gizmo(_e):
                n = self._obj_nodes.get(o.id)
                if n is None:
                    return
                if giz.value and n["gizmo"] is None:
                    w, p = tf_to_viser(tf_from_xyz_rpy_deg(o.pose.xyz, o.pose.rpy_deg))
                    tc = self.server.scene.add_transform_controls(f"/gizmos/obj_{o.id}", scale=0.25,
                                                                  wxyz=w, position=p)
                    tc.on_update(on_gizmo)
                    n["gizmo"] = tc
                elif not giz.value and n["gizmo"] is not None:
                    n["gizmo"].remove()
                    n["gizmo"] = None

            def on_vis(_e):
                o.visible = vis.value
                n = self._obj_nodes.get(o.id)
                if n and n["mesh"] is not None:
                    n["mesh"].visible = o.visible

            def on_color(_e):
                o.color = _rgb_to_hex(col.value)
                n = self._obj_nodes.get(o.id)
                if n and n["mesh"] is not None:
                    n["mesh"].color = col.value

            def on_opacity(_e):
                o.opacity = float(opa.value)
                n = self._obj_nodes.get(o.id)
                if n and n["mesh"] is not None:
                    n["mesh"].opacity = o.opacity

            def on_remove(_e):
                with self.scene_lock:
                    self.scene.objects = [x for x in self.scene.objects if x.id != o.id]
                    self._remove_obj_3d(o.id)
                self._rebuild_scene_gui()

            pos.on_update(from_fields)
            rot.on_update(from_fields)
            giz.on_update(toggle_gizmo)
            vis.on_update(on_vis)
            col.on_update(on_color)
            opa.on_update(on_opacity)
            rm_.on_click(on_remove)

    # -- toolhead
    def _build_toolhead_gui(self):
        g = self.server.gui
        th = self.scene.toolhead
        # --- library presets (toolheads/<id>/), shared by all scenes
        g.add_markdown("**Library** — presets in `toolheads/<id>/` (mesh + offset + TCP). "
                       "Using one copies it into this scene.")
        lib = g.add_dropdown("Toolhead preset", [""] + sc.list_toolheads(),
                             initial_value=th.library if th.library in sc.list_toolheads() else "")
        b_use = g.add_button("Use selected preset", icon=viser.Icon.TOOL)
        lib_name = g.add_text("Preset name", th.library or "")
        b_store = g.add_button("Save current toolhead to library", hint="Writes toolheads/<name>/")

        def use_preset(_e):
            if not lib.value:
                return self._notify("pick a preset first", error=True)
            with self.scene_lock:
                try:
                    sc.apply_toolhead_preset(self.scene, lib.value)
                except Exception as e:
                    return self._notify(f"preset failed: {e}", error=True)
                self._apply_toolhead_3d()
            self._notify(f"toolhead preset {lib.value!r} applied (Save the scene to keep it)")
            self._rebuild_scene_gui()

        def store_preset(_e):
            try:
                path = sc.save_toolhead_preset(self.scene, lib_name.value)
            except Exception as e:
                return self._notify(f"save to library failed: {e}", error=True)
            self._notify(f"saved {path}")
            self._rebuild_scene_gui()

        b_use.on_click(use_preset)
        b_store.on_click(store_preset)
        if th.library:
            g.add_markdown(f"Current toolhead comes from preset `{th.library}`.")
        g.add_markdown("TCP offset in **tool0** (flange) frame — the same numbers you enter in "
                       "PolyScope *Installation → General → TCP* (mm, rotation vector rad).")
        tcp_xyz = g.add_vector3("TCP position [mm]", tuple(1000 * v for v in th.tcp_xyz), step=0.5)
        tcp_rv = g.add_vector3("TCP rotation vector [rad]", tuple(th.tcp_rotvec_rad), step=0.01)
        tcp_rpy = g.add_vector3("same as RPY [deg] (read-only)", tuple(
            float(v) for v in np.degrees(rot_to_rpy(rotvec_to_rot(th.tcp_rotvec_rad)))), disabled=True)
        payload = g.add_number("Payload [kg] (documented only)", th.payload_kg, min=0.0, max=5.0, step=0.01,
                               hint="Stored in the scene for the record; the viewer does nothing with it")
        g.add_markdown("Toolhead mesh (optional), attached to tool0:")
        units = g.add_dropdown("Units of the file", ["mm", "m"], initial_value=th.mesh_units)
        up = g.add_upload_button("Upload toolhead STL / OBJ / GLB", icon=viser.Icon.UPLOAD,
                                 mime_type=".stl,.obj,.glb,.gltf,.ply")
        existing = g.add_dropdown("Mesh file", [""] + sc.list_models(self.scene.name),
                                  initial_value=th.mesh or "")
        moff_p = g.add_vector3("Mesh offset [mm]", tuple(1000 * v for v in th.mesh_offset.xyz), step=0.5)
        moff_r = g.add_vector3("Mesh offset RPY [deg]", tuple(th.mesh_offset.rpy_deg), step=1.0)
        col = g.add_rgb("Colour", _hex_to_rgb(th.color))
        clear = g.add_button("Remove toolhead mesh")

        def tcp_changed(_e=None):
            th.library = None
            th.tcp_xyz = [v / 1000.0 for v in tcp_xyz.value]
            th.tcp_rotvec_rad = list(tcp_rv.value)
            tcp_rpy.value = tuple(round(float(v), 2) for v in np.degrees(rot_to_rpy(rotvec_to_rot(th.tcp_rotvec_rad))))
            self._apply_toolhead_3d()

        def mesh_changed(_e=None):
            th.library = None
            th.mesh_units = units.value
            th.mesh = existing.value or None
            th.mesh_offset.xyz = [v / 1000.0 for v in moff_p.value]
            th.mesh_offset.rpy_deg = list(moff_r.value)
            th.color = _rgb_to_hex(col.value)
            self._apply_toolhead_3d()

        def on_upload(ev):
            f = ev.target.value
            try:
                rel = sc.add_model_file(self.scene.name, f.name, f.content)
            except Exception as e:
                return self._notify(f"upload failed: {e}", error=True)
            existing.options = [""] + sc.list_models(self.scene.name)
            existing.value = rel
            mesh_changed()

        def on_clear(_e):
            existing.value = ""
            mesh_changed()

        tcp_xyz.on_update(tcp_changed)
        tcp_rv.on_update(tcp_changed)
        payload.on_update(lambda _e: setattr(th, "payload_kg", float(payload.value)))
        for h in (units, existing, moff_p, moff_r, col):
            h.on_update(mesh_changed)
        up.on_upload(on_upload)
        clear.on_click(on_clear)

    # -- features
    def _build_add_feature_gui(self):
        g = self.server.gui
        g.add_markdown("A *plane feature* = a frame in Base whose **Z axis** becomes a safety-plane "
                       "normal. Create the same feature on the pendant (*Installation → Features*).")
        name = g.add_text("New feature name", "")
        b = g.add_button("Add feature at Base origin")

        def add(_e):
            fid = sc.slug(name.value)
            if not fid or fid in (sc.FEATURE_BASE, sc.FEATURE_TOOL):
                return self._notify("give the feature a name (not 'base'/'tool')", error=True)
            with self.scene_lock:
                fid = self.scene.unique_id(fid, {f.id for f in self.scene.features})
                f = sc.Feature(id=fid)
                self.scene.features.append(f)
                self._add_feat_3d(f)
            self._rebuild_scene_gui()

        b.on_click(add)

    def _build_feature_gui(self, f: sc.Feature):
        g = self.server.gui
        with g.add_folder(f"feature: {f.id}", expand_by_default=False):
            giz = g.add_checkbox("Gizmo (drag in 3D)", False)
            pos = g.add_vector3("Position [mm]", tuple(1000 * v for v in f.pose.xyz), step=1.0)
            rot = g.add_vector3("Rotation RPY [deg]", tuple(f.pose.rpy_deg), step=1.0)
            rv = g.add_vector3("same as rotation vector [rad]", tuple(
                float(v) for v in rot_to_rotvec(rpy_to_rot(np.radians(f.pose.rpy_deg)))), disabled=True)
            rm_ = g.add_button("Remove feature", color="red")

            def push():
                rv.value = tuple(round(float(v), 4) for v in rot_to_rotvec(rpy_to_rot(np.radians(f.pose.rpy_deg))))

            def from_fields(_e=None):
                if self._syncing:
                    return
                f.pose.xyz = [v / 1000.0 for v in pos.value]
                f.pose.rpy_deg = list(rot.value)
                push()
                self._set_feat_pose_3d(f)

            def on_gizmo(ev):
                tc = ev.target
                f.pose.xyz = [float(v) for v in tc.position]
                f.pose.rpy_deg = [float(v) for v in np.degrees(rot_to_rpy(quat_wxyz_to_rot(tc.wxyz)))]
                self._set_feat_pose_3d(f, move_gizmo=False)
                self._syncing = True
                try:
                    pos.value = tuple(1000 * v for v in f.pose.xyz)
                    rot.value = tuple(round(v, 2) for v in f.pose.rpy_deg)
                    push()
                finally:
                    self._syncing = False

            def toggle_gizmo(_e):
                n = self._feat_nodes.get(f.id)
                if n is None:
                    return
                if giz.value and n["gizmo"] is None:
                    w, p = tf_to_viser(tf_from_xyz_rpy_deg(f.pose.xyz, f.pose.rpy_deg))
                    tc = self.server.scene.add_transform_controls(f"/gizmos/feat_{f.id}", scale=0.2,
                                                                  wxyz=w, position=p)
                    tc.on_update(on_gizmo)
                    n["gizmo"] = tc
                elif not giz.value and n["gizmo"] is not None:
                    n["gizmo"].remove()
                    n["gizmo"] = None

            def on_remove(_e):
                used = [p.name for p in self.scene.safety_planes if p.feature == f.id]
                if used:
                    return self._notify(f"feature {f.id!r} is used by plane(s) {used} — change them first",
                                        error=True)
                with self.scene_lock:
                    self.scene.features = [x for x in self.scene.features if x.id != f.id]
                    self._remove_feat_3d(f.id)
                self._rebuild_scene_gui()

            pos.on_update(from_fields)
            rot.on_update(from_fields)
            giz.on_update(toggle_gizmo)
            rm_.on_click(on_remove)

    # -- planes
    def _feature_options(self):
        return [sc.FEATURE_BASE] + [f.id for f in self.scene.features]

    def _build_add_plane_gui(self):
        g = self.server.gui
        g.add_markdown(
            "Same fields as *Installation → Safety → Planes*: copy a feature, displacement along its Z, "
            "restriction mode, restrict elbow. **Arrow = allowed side.** Nothing is enforced here.")
        name = g.add_text("New plane name", "")
        b = g.add_button("Add plane")

        def add(_e):
            pname = sc.slug(name.value)
            if not pname:
                return self._notify("give the plane a name", error=True)
            if len(self.scene.safety_planes) >= sp.MAX_PLANES:
                return self._notify(f"PolyScope allows at most {sp.MAX_PLANES} planes", error=True)
            with self.scene_lock:
                pname = self.scene.unique_id(pname, {p.name for p in self.scene.safety_planes})
                p = sc.SafetyPlane(name=pname)
                self.scene.safety_planes.append(p)
                self._add_plane_3d(p)
            self._rebuild_scene_gui()

        b.on_click(add)

    def _build_plane_gui(self, p: sc.SafetyPlane):
        g = self.server.gui
        with g.add_folder(f"plane: {p.name}", expand_by_default=False):
            status = g.add_text("Status", "", disabled=True)
            self._plane_status_text[p.name] = status
            feat = g.add_dropdown("Copy feature", self._feature_options(),
                                  initial_value=p.feature if p.feature in self._feature_options() else sc.FEATURE_BASE)
            disp = g.add_number("Displacement [mm]", round(1000 * p.displacement, 2), step=1.0,
                                hint="Along the feature's Z axis; the plane normal is that Z")
            mode = g.add_dropdown("Restriction", [sc.PLANE_MODE_LABELS[m] for m in sc.PLANE_MODES],
                                  initial_value=sc.PLANE_MODE_LABELS[p.mode])
            elbow = g.add_checkbox("Restrict Elbow", p.restrict_elbow)
            vis = g.add_checkbox("Show", p.visible)
            size = g.add_vector3("Drawn size [m] (x, y, –)", (p.size[0], p.size[1], 0.0), step=0.1,
                                 hint="Rendering only; a real plane is infinite")
            rm_ = g.add_button("Remove plane", color="red")

            def pose_changed(_e=None):
                p.feature = feat.value
                p.displacement = float(disp.value) / 1000.0
                p.visible = vis.value
                self._set_plane_pose_3d(p)

            def geom_changed(_e=None):
                inv = {v: k for k, v in sc.PLANE_MODE_LABELS.items()}
                p.mode = inv[mode.value]
                p.restrict_elbow = elbow.value
                p.size = [max(0.1, float(size.value[0])), max(0.1, float(size.value[1]))]
                self._refresh_plane_3d(p)

            def on_remove(_e):
                with self.scene_lock:
                    self.scene.safety_planes = [x for x in self.scene.safety_planes if x.name != p.name]
                    self._remove_plane_3d(p.name)
                self._rebuild_scene_gui()

            for h in (feat, disp, vis):
                h.on_update(pose_changed)
            for h in (mode, elbow, size):
                h.on_update(geom_changed)
            rm_.on_click(on_remove)

    # ------------------------------------------------------------ live loop
    def rtde_thread(self, host: str, rate: float):
        """Receive-only RTDE poll loop with a staleness watchdog.

        ur_rtde's RTDEReceiveInterface keeps returning the *last* values if the
        stream stops (e.g. the controller re-syncs RTDE when another client
        connects) instead of raising. So we also watch the robot timestamp: if
        it does not advance for STALL_S seconds, or the interface reports
        disconnected, we drop the connection and reconnect.
        """
        from ur5e.connection import RobotConnection   # local: ur_rtde only needed online
        period = 1.0 / rate
        STALL_S = 1.5
        while not self._stop:
            try:
                with RobotConnection(host) as r:
                    self.rtde_status = f"RTDE: connected to {host} ({rate:.0f} Hz, receive-only)"
                    last_ts, last_change = None, time.time()
                    while not self._stop:
                        rx = r.receive
                        q = rx.getActualQ()
                        tcp = rx.getActualTCPPose()
                        ts = rx.getTimestamp() if hasattr(rx, "getTimestamp") else None
                        now = time.time()
                        if not q or len(q) != 6:
                            raise RuntimeError("empty joint state (robot booting?)")
                        if hasattr(rx, "isConnected") and not rx.isConnected():
                            raise RuntimeError("receive interface disconnected")
                        if ts is not None:
                            if ts != last_ts:
                                last_ts, last_change = ts, now
                            elif now - last_change > STALL_S:
                                raise RuntimeError(f"RTDE stream stalled for {STALL_S} s (no new robot timestamp)")
                        with self.lock:
                            self.q = np.asarray(q, dtype=float)
                            self.controller_tcp = np.asarray(tcp, dtype=float) if tcp and len(tcp) == 6 else None
                            self.last_rx = now
                            self.robot_ts = ts
                        time.sleep(period)
            except Exception as e:  # noqa: BLE001 — keep the viewer alive, retry
                msg = f"{type(e).__name__}: {str(e)[:70]}"
                print(f"[viewer] RTDE: {msg} — reconnecting to {host} in 2 s")
                self.rtde_status = f"RTDE: reconnecting to {host} … ({msg})"
                for _ in range(20):
                    if self._stop:
                        break
                    time.sleep(0.1)

    def run(self, rate: float):
        period = 1.0 / rate
        n = 0
        try:
            while not self._stop:
                t0 = time.time()
                with self.lock:
                    q = self.q.copy()
                    ctcp = None if self.controller_tcp is None else self.controller_tcp.copy()
                try:
                    frames = rm.fk(q)
                    self._update_robot(frames)
                    th = self.scene.toolhead
                    T_tcp = frames["tool0"] @ make_tf(rotvec_to_rot(th.tcp_rotvec_rad), th.tcp_xyz)
                    self._update_planes(frames, T_tcp[:3, 3])
                    if n % max(1, int(rate / 4)) == 0:
                        self._update_readouts(q, frames, T_tcp, ctcp)
                except Exception:
                    traceback.print_exc()
                n += 1
                time.sleep(max(0.0, period - (time.time() - t0)))
        except KeyboardInterrupt:
            pass
        finally:
            self._stop = True

    def _update_planes(self, frames, tcp_world):
        with self.scene_lock:
            try:
                st = sp.evaluate(self.scene, frames, tcp_world)
            except Exception:
                return
            for p in self.scene.safety_planes:
                s = st.get(p.name)
                col = sp.render_color(p, s)
                n = self._plane_nodes.get(p.name)
                if n and self._last_plane_colors.get(p.name) != col:
                    n["box"].color = col
                    n["arrow"].color = col
                    self._last_plane_colors[p.name] = col
                txt = self._plane_status_text.get(p.name)
                if txt is not None and s is not None:
                    parts = [f"TCP {1000 * s.tcp_d:+.0f} mm {'✓' if s.tcp_d >= 0 else '✗'}"]
                    if s.elbow_d is not None:
                        parts.append(f"elbow {1000 * s.elbow_d:+.0f} mm {'✓' if s.elbow_d >= 0 else '✗'}")
                    if s.violated:
                        parts.append("— CROSSED")
                    elif s.triggered:
                        parts.append("— would trigger Reduced mode")
                    new = " · ".join(parts)
                    if txt.value != new:
                        txt.value = new

    def _update_readouts(self, q, frames, T_tcp, ctcp):
        status = self.rtde_status
        if not self.offline and self.last_rx > 0:
            age = time.time() - self.last_rx
            ts = f", robot t={self.robot_ts:.1f} s" if isinstance(self.robot_ts, (int, float)) else ""
            status += f"  [last update {age:.1f} s ago{ts}]"
        if self._st_rtde.value != status:
            self._st_rtde.value = status
        self._st_q.value = "  ".join(f"{math.degrees(v):7.2f}" for v in q)
        view_pose = np.concatenate([T_tcp[:3, 3], rot_to_rotvec(T_tcp[:3, :3])])
        self._st_tcp_view.value = f"{_fmt_mm(view_pose[:3])} | " + " ".join(f"{v:6.3f}" for v in view_pose[3:])
        if ctcp is None:
            self._st_tcp_ctrl.value = "—"
            self._st_match.value = "no controller data"
            return
        self._st_tcp_ctrl.value = f"{_fmt_mm(ctcp[:3])} | " + " ".join(f"{v:6.3f}" for v in ctcp[3:])
        d_mm = 1000.0 * float(np.linalg.norm(ctcp[:3] - view_pose[:3]))
        if d_mm < 3.0:
            self._st_match.value = f"✓ scene TCP matches controller (Δ {d_mm:.1f} mm)"
        else:
            self._st_match.value = (f"⚠ Δ {d_mm:.1f} mm — the TCP in this scene differs from the TCP "
                                    f"active on the controller (Installation → TCP)")


def run_viewer(*, host: Optional[str], scene_name: str, rate: float = 20.0,
               port: int = 7000, bind: str = "0.0.0.0", joints_deg=None) -> None:
    """Entry point used by scripts/rtde_viewer.py. host=None -> offline mode."""
    server = viser.ViserServer(host=bind, port=port, label="UR5e viewer", verbose=False)
    app = ViewerApp(server, scene_name, offline=host is None)
    if joints_deg is not None and host is None:
        app.q = np.radians(joints_deg)
        for h, v in zip(getattr(app, "_sliders", []), joints_deg):
            h.value = float(v)
    print(f"viewer ready -> open http://localhost:7001/ in your browser "
          f"(container port {port}); scene folder: {sc.scenes_root()}")
    if host is not None:
        threading.Thread(target=app.rtde_thread, args=(host, rate), daemon=True).start()
        print(f"streaming joint state from {host} (Ctrl-C to stop) ...")
    else:
        print("offline mode: no RTDE, pose the arm with the joint sliders")
    try:
        app.run(rate)
    finally:
        server.stop()
