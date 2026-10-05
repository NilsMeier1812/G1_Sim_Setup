#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scene_markers.py — gemeinsames Wire-Format zwischen scene_bridge (Erzeuger von
/scene_markers) und dessen Konsumenten (create_map.py fuer die Nav-Karte,
ik_solver.py fuer die Umgebungs-Kollision). visualization_msgs/MarkerArray ist
bewusst das EINZIGE Format (kein eigener .msg-Typ, kein moveit_msgs) -- RViz
kann es nativ anzeigen, und beide Konsumenten koennen denselben Topic
abonnieren, statt Szenen-Daten mehrfach zu parsen/uebertragen.

Kontrakt pro Marker:
  - ns    = NS_OBSTACLE oder NS_GRASP                  (Hindernis vs. Greif-
            Objekt -- steuert im IK-Solver die Kollisions-Ausnahme, siehe
            g1pilot/docs/11_arm_manipulation_technik.md (Umgebungs-Kollisionsgate))
            oder NS_DISPLAY (nur Anzeige in RViz, z.B. das Optik-Mesh eines in
            konvexe Teile zerlegten Arbeitsplatzes -- IK/Nav ignorieren es, die
            Teile kommen als eigene NS_OBSTACLE-Boxen, siehe is_collision_marker)
  - id    = stable_id(name)                            (bleibt ueber Ticks
            stabil, damit RViz Marker nicht staendig neu anlegt)
  - text  = encode_text(name, aabb_half) / decode_text(...)  (wiederverwendetes
            Feld -- traegt den Original-Objektnamen und, nur bei Mesh-Geomen,
            die lokale Bounding-Box VOR Rotation/Translation. Analog zur
            reserve[]-Wiederverwendung in unitree_sdk2py_bridge.py.)
  - type/scale/pose/color: Standard-RViz-Semantik (siehe marker_type_for()/
    marker_scale_for() fuer die MuJoCo->Marker-Umrechnung).
"""
import zlib

from visualization_msgs.msg import Marker

NS_OBSTACLE = "g1scene:obstacle"
NS_GRASP = "g1scene:grasp"
NS_DISPLAY = "g1scene:display"

_MJ_TYPE_TO_MARKER = {
    "box": Marker.CUBE,
    "sphere": Marker.SPHERE,
    "cylinder": Marker.CYLINDER,
    "mesh": Marker.MESH_RESOURCE,
}


def ns_for_class(cls: str) -> str:
    return {"grasp": NS_GRASP, "display": NS_DISPLAY}.get(cls, NS_OBSTACLE)


def class_from_ns(ns: str) -> str:
    return "grasp" if ns == NS_GRASP else "obstacle"


def is_collision_marker(marker) -> bool:
    """False fuer reine Anzeige-Marker (NS_DISPLAY) -- IK und Nav-Karte
    ueberspringen sie."""
    return marker.ns != NS_DISPLAY


def stable_id(name: str) -> int:
    """Deterministische, stabile Marker-ID aus dem Objekt-Namen (statt eines
    Laufindex, der sich bei Hinzufuegen/Entfernen anderer Objekte verschieben
    wuerde -> Marker wuerden in RViz fuer die falschen Objekte 'springen')."""
    return zlib.crc32(name.encode("utf-8")) & 0x7FFFFFFF


def encode_text(name: str, aabb_half=None) -> str:
    if aabb_half is None:
        return name
    return f"{name}|{aabb_half[0]:.5f}|{aabb_half[1]:.5f}|{aabb_half[2]:.5f}"


def decode_text(text: str):
    """-> (name, aabb_half_or_None). Robust gegen leere/kaputte Strings."""
    parts = (text or "").split("|")
    if len(parts) == 4:
        try:
            return parts[0], (float(parts[1]), float(parts[2]), float(parts[3]))
        except ValueError:
            return parts[0], None
    return (parts[0] if parts else ""), None


def marker_type_for(mj_type: str):
    return _MJ_TYPE_TO_MARKER.get(mj_type, Marker.CUBE)


def marker_scale_for(mj_type: str, size):
    """MuJoCo-Groesse -> RViz-Marker-Skalierung. MuJoCo/Marker verwenden fuer
    dieselbe Form UNTERSCHIEDLICHE Konventionen (MuJoCo box=Halbextents,
    Marker-CUBE=Vollextents; MuJoCo sphere/cylinder-Groesse=Radius,
    Marker-Durchmesser)."""
    if mj_type == "box" and size and len(size) >= 3:
        return (2.0 * size[0], 2.0 * size[1], 2.0 * size[2])
    if mj_type == "sphere" and size:
        d = 2.0 * size[0]
        return (d, d, d)
    if mj_type == "cylinder" and size and len(size) >= 2:
        d = 2.0 * size[0]
        h = 2.0 * size[1]
        return (d, d, h)
    if mj_type == "mesh":
        # MJCF-<mesh scale="..">-Faktoren direkt uebernehmen: Marker skaliert
        # MESH_RESOURCE-Ressourcen mit derselben Semantik (Multiplikator auf
        # die im Ressourcen-File gespeicherten Vertex-Koordinaten).
        return tuple(size[:3]) if size and len(size) >= 3 else (1.0, 1.0, 1.0)
    return (0.1, 0.1, 0.1)


def local_half_extents_from_marker(marker) -> tuple:
    """Objekt-lokale (unrotierte) Halbextents direkt aus einem bereits
    gebauten Marker -- fuer Fussabdruck-/Kollisions-Naeherungen (2D-Nav-
    Rasterung in create_map.py, 3D-Box-Proxy im IK-Solver). Arbeitet
    bewusst NUR mit Marker-Feldern (type/scale/text), nicht mit den
    urspruenglichen MuJoCo-Rohwerten -- Konsumenten muessen so nur EIN Format
    (das Wire-Format /scene_markers) verstehen.

    CUBE: scale = volle Kantenlaenge -> halbieren.
    SPHERE: scale.x = Durchmesser -> halbieren (gleich in alle Achsen).
    CYLINDER: scale.x/y = Durchmesser, scale.z = volle Hoehe.
    MESH_RESOURCE: scale ist KEIN Groessenmass (MJCF-Mesh-Skalierungsfaktor) --
    hier zaehlt die im Marker.text mitgefuehrte AABB (siehe decode_text()).
    Ohne AABB (z.B. alter/kaputter Snapshot) eine kleine, konservative
    Default-Box.
    """
    mtype = marker.type
    sx, sy, sz = marker.scale.x, marker.scale.y, marker.scale.z
    if mtype == Marker.CUBE:
        return (sx / 2.0, sy / 2.0, sz / 2.0)
    if mtype == Marker.SPHERE:
        r = sx / 2.0
        return (r, r, r)
    if mtype == Marker.CYLINDER:
        r = sx / 2.0
        return (r, r, sz / 2.0)
    if mtype == Marker.MESH_RESOURCE:
        _, aabb_half = decode_text(marker.text)
        if aabb_half is not None:
            return aabb_half
        return (0.1, 0.1, 0.1)
    return (0.1, 0.1, 0.1)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q_wxyz, v):
    """Rotiert v=(x,y,z) mit Quaternion q=(w,x,y,z). Pure Python (kein numpy),
    damit dieses Modul auch ausserhalb eines ROS/numpy-Envs lesbar/testbar
    bleibt (nur Marker-Import braucht ROS)."""
    w, x, y, z = q_wxyz
    qv = (0.0, v[0], v[1], v[2])
    qc = (w, -x, -y, -z)
    _, rx, ry, rz = _quat_mul(_quat_mul(q_wxyz, qv), qc)
    return (rx, ry, rz)


def footprint_xy(marker):
    """Welt-XY-Position + achsenparallele Halbextents eines Markers -- alles,
    was create_map.py braucht, um ein Rechteck in die Occupancy-Grid zu
    stempeln. Rueckgabe: (x, y, hx, hy)."""
    q = marker.pose.orientation
    local_half = local_half_extents_from_marker(marker)
    hx, hy = world_xy_half_extent((q.w, q.x, q.y, q.z), local_half)
    return marker.pose.position.x, marker.pose.position.y, hx, hy


def vertical_range(marker):
    """Welt-Z-Bereich (z_min, z_max) eines Markers (konservative Box)."""
    q = marker.pose.orientation
    hz = world_half_extents((q.w, q.x, q.y, q.z), local_half_extents_from_marker(marker))[2]
    z = marker.pose.position.z
    return z - hz, z + hz


def world_half_extents(quat_wxyz, local_half):
    """Welt-achsenparallele Halbextents (hx, hy, hz) einer gedrehten Box."""
    w, x, y, z = quat_wxyz
    R = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(abs(R[i][k]) * local_half[k] for k in range(3)) for i in range(3))


def world_xy_half_extent(quat_wxyz, local_half):
    """Welt-achsenparallele XY-Halbextents einer (evtl. gedrehten) lokalen Box
    -- konservative AABB-Naeherung (siehe g1pilot/docs/51_navigation_technik.md, Mesh-
    Handling: 'Bounding-Box statt exaktem Konvex-Huelle-Fussabdruck')."""
    hx, hy, hz = local_half
    xs = []
    ys = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                rx, ry, _ = quat_rotate(quat_wxyz, (sx * hx, sy * hy, sz * hz))
                xs.append(rx)
                ys.append(ry)
    return (max(abs(min(xs)), abs(max(xs))), max(abs(min(ys)), abs(max(ys))))
