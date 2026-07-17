"""3D visualization of the robot's state estimate for viser.

Builds the kinematic tree and visual meshes directly from the ground-truth
MJCF (pineappleV3/pineappleV3_mjcf/pineappleV3_armless.xml, vendored inside
motor_control) using stdlib XML parsing + trimesh for the .obj files — no
mujoco dependency on the Pi. Falls back to a primitive box model if the
MJCF or trimesh are unavailable.

Driven by the state estimator: base tilt (yaw stripped so the robot stays
centered), base height, and calibrated joint angles.
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET

import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(BASE_DIR, "pineappleV3", "pineappleV3_mjcf",
                         "pineappleV3_armless.xml")

STANDING_BASE_HEIGHT = 0.3807

# joint-index (runtime order) -> (mjcf body containing the joint, axis)
JOINT_BODY = {
    0: ("hip_thigh_conn_l", "x"), 1: ("thigh_l", "y"),
    2: ("calf_l", "y"), 3: ("wheel_l", "y"),
    4: ("hip_thigh_conn_r", "x"), 5: ("thigh_r", "y"),
    6: ("calf_r", "y"), 7: ("wheel_r", "y"),
}


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def rx(a):
    return np.array([np.cos(a / 2), np.sin(a / 2), 0.0, 0.0])


def ry(a):
    return np.array([np.cos(a / 2), 0.0, np.sin(a / 2), 0.0])


def yaw_stripped(quat):
    w, x, y, z = quat
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    qz_inv = np.array([np.cos(-yaw / 2), 0.0, 0.0, np.sin(-yaw / 2)])
    q = _qmul(qz_inv, quat)
    return q / np.linalg.norm(q)


def _vec(s, default):
    return np.array([float(v) for v in s.split()]) if s else np.array(default)


def _material_color(name):
    m = re.search(r"\((\d+),(\d+),(\d+)\)", name or "")
    if m:
        return tuple(int(v) for v in m.groups())
    return (150, 150, 155)


class _MjcfModel:
    """Body tree + visual mesh geoms parsed from the MJCF."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        meshdir = root.find("compiler").get("meshdir", "meshes/")
        self.mesh_files = {}
        for mesh in root.iter("mesh"):
            f = mesh.get("file")
            name = mesh.get("name") or os.path.splitext(os.path.basename(f))[0]
            self.mesh_files[name] = os.path.join(os.path.dirname(path),
                                                 meshdir, f)
        self.bodies = {}  # name -> (parent, pos, quat, [visual geoms])
        world = root.find("worldbody")

        def walk(el, parent):
            for body in el.findall("body"):
                name = body.get("name")
                pos = _vec(body.get("pos"), [0, 0, 0])
                quat = _vec(body.get("quat"), [1, 0, 0, 0])
                geoms = []
                for g in body.findall("geom"):
                    if g.get("class") == "visual" and g.get("mesh"):
                        geoms.append((
                            g.get("mesh"),
                            _vec(g.get("pos"), [0, 0, 0]),
                            _vec(g.get("quat"), [1, 0, 0, 0]),
                            _material_color(g.get("material")),
                        ))
                self.bodies[name] = (parent, pos, quat, geoms)
                walk(body, name)

        walk(world, None)

    def path(self, name):
        segs = []
        cur = name
        while cur is not None:
            segs.append(cur)
            cur = self.bodies[cur][0]
        return "/robot/" + "/".join(reversed(segs[:-1])) if len(segs) > 1 \
            else "/robot"


class RobotViz:
    def __init__(self, server):
        s = server.scene
        s.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)
        self.base = s.add_frame("/robot", show_axes=False,
                                position=(0, 0, STANDING_BASE_HEIGHT))
        # axes triad at the floating-base frame origin (the MJCF base_link
        # origin, where the IMU site sits): red +x fwd, green +y left,
        # blue +z up
        s.add_frame("/robot/heading", show_axes=True, axes_length=0.22,
                    axes_radius=0.004, position=(0.0, 0.0, 0.0))
        self.joints = {}
        try:
            self._build_from_mjcf(s)
            print(f"robot viz: meshes from {MJCF_PATH}")
        except Exception as e:
            print(f"robot viz: MJCF meshes unavailable ({e}) — primitive model")
            self._build_primitives(s)

    def _build_from_mjcf(self, s):
        import trimesh

        model = _MjcfModel(MJCF_PATH)
        mesh_cache = {}
        for name, (parent, pos, quat, geoms) in model.bodies.items():
            if name == "base_link":
                frame_path = "/robot"
            else:
                frame_path = model.path(name)
                frame = s.add_frame(frame_path, show_axes=False,
                                    position=pos, wxyz=quat)
                for j, (body, _axis) in JOINT_BODY.items():
                    if body == name:
                        self.joints[j] = (frame, quat.copy())
            for gi, (mesh_name, gpos, gquat, color) in enumerate(geoms):
                f = model.mesh_files[mesh_name]
                if f not in mesh_cache:
                    m = trimesh.load(f, force="mesh")
                    mesh_cache[f] = (np.asarray(m.vertices, dtype=np.float32),
                                     np.asarray(m.faces, dtype=np.uint32))
                v, fc = mesh_cache[f]
                s.add_mesh_simple(f"{frame_path}/geom_{gi}", vertices=v,
                                  faces=fc, color=color, position=gpos,
                                  wxyz=gquat, flat_shading=False)

    def _build_primitives(self, s):
        HIP_POS = np.array([0.0303, 0.1208, 0.04299])
        THIGH_POS = np.array([-0.04, -0.034, 0.0])
        CALF_POS = np.array([0.0, 0.0741, -0.225])
        WHEEL_POS = np.array([0.0, 0.0563, -0.225])
        s.add_box("/robot/chassis", dimensions=(0.24, 0.30, 0.12),
                  position=(0.0, 0.0, 0.05), color=(220, 170, 40))
        for side, sgn, jbase in (("l", 1.0, 0), ("r", -1.0, 4)):
            flip = np.array([1.0, sgn, 1.0])
            hip = s.add_frame(f"/robot/hip_{side}", show_axes=False,
                              position=HIP_POS * flip)
            thigh = s.add_frame(f"/robot/hip_{side}/thigh", show_axes=False,
                                position=THIGH_POS * flip)
            s.add_box(f"/robot/hip_{side}/thigh/geom",
                      dimensions=(0.055, 0.05, 0.235),
                      position=(0, 0.037 * sgn, -0.1125), color=(90, 120, 190))
            calf = s.add_frame(f"/robot/hip_{side}/thigh/calf",
                               show_axes=False, position=CALF_POS * flip)
            s.add_box(f"/robot/hip_{side}/thigh/calf/geom",
                      dimensions=(0.045, 0.04, 0.235),
                      position=(0, 0.028 * sgn, -0.1125), color=(90, 170, 120))
            wheel = s.add_frame(f"/robot/hip_{side}/thigh/calf/wheel",
                                show_axes=False, position=WHEEL_POS * flip)
            ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            ring = np.stack([0.0925 * np.cos(ang), np.zeros(24),
                             0.0925 * np.sin(ang)], 1)
            yo = 0.0097 * sgn
            verts = np.vstack([ring + [0, yo - 0.0175, 0],
                               ring + [0, yo + 0.0175, 0],
                               [[0, yo - 0.0175, 0]], [[0, yo + 0.0175, 0]]]
                              ).astype(np.float32)
            faces = []
            for i in range(24):
                j = (i + 1) % 24
                faces += [[i, j, 24 + i], [j, 24 + j, 24 + i],
                          [48, j, i], [49, 24 + i, 24 + j]]
            s.add_mesh_simple(f"/robot/hip_{side}/thigh/calf/wheel/geom",
                              vertices=verts,
                              faces=np.array(faces, dtype=np.uint32),
                              color=(60, 60, 65), flat_shading=True)
            s.add_box(f"/robot/hip_{side}/thigh/calf/wheel/mark",
                      dimensions=(0.02, 0.04, 0.02), position=(0.07, yo, 0),
                      color=(230, 60, 60))
            ident = np.array([1.0, 0, 0, 0])
            self.joints[jbase + 0] = (hip, ident)
            self.joints[jbase + 1] = (thigh, ident)
            self.joints[jbase + 2] = (calf, ident)
            self.joints[jbase + 3] = (wheel, ident)

    def update(self, quat, z_err, q_joints):
        """quat: base orientation (yaw arbitrary); z_err: height error [m];
        q_joints: (8,) sim-frame [l_hip, l_thigh, l_calf, l_wheel, r_...]."""
        q = yaw_stripped(np.asarray(quat, float))
        self.base.wxyz = q
        self.base.position = (0.0, 0.0, STANDING_BASE_HEIGHT + float(z_err))
        for j, (frame, static) in self.joints.items():
            angle = float(q_joints[j])
            jq = rx(angle) if JOINT_BODY[j][1] == "x" else ry(angle)
            frame.wxyz = _qmul(static, jq)
