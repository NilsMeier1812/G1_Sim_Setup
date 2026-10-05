#!/usr/bin/env python3
"""
CAD-/Mesh-Import fuer den Scene Editor: STEP/IGES/BREP und fremde Mesh-Formate
-> MuJoCo-taugliche Einzelteil-Meshes + fertige Umgebung.

Warum Einzelteile statt EINES grossen Netzes?
---------------------------------------------
Frueher wurde eine STEP-Datei komplett zu *einer* STL tesseliert. Fuer ein
einzelnes Bauteil geht das, fuer eine Baugruppe (Roboterzelle, Anlage) nicht:

  * Eine Baugruppe hat schnell Millionen Dreiecke - MuJoCo laedt STL aber nur
    bis 200000 Dreiecke. Vergroebern hilft da nicht, es macht das Modell nur
    haesslich.
  * MuJoCo kollidiert mit Meshes ueber deren KONVEXE HUELLE. Eine ganze Zelle
    als ein Mesh waere ein einziger konvexer Klotz - der Roboter koennte gar
    nicht hinein.

Darum wird die Baugruppe hier so zerlegt, wie sie im CAD aufgebaut ist:

  * je Bauteil (genauer: je Volumenkoerper eines Bauteils) ein eigenes Mesh,
    jeweils auf seine Bounding-Box-Mitte zentriert (die Abnehmer - RViz, Nav,
    IK - rechnen mit einer Box um den Geom-Ursprung);
  * Bauteile, die mehrfach verbaut sind (Schrauben, Profile, ...), werden nur
    EINMAL gespeichert und mehrfach platziert;
  * Namen, Farben und Lage kommen aus dem CAD; die Einheit wird aus der Datei
    gelesen (mm/inch/m -> Meter), gespiegelte Instanzen werden korrekt
    eingebacken (MuJoCo-Quaternionen koennen nicht spiegeln);
  * sehr kleine Teile (Schrauben, Muttern, Knoepfe) werden nur angezeigt und
    nehmen nicht an der Kollision teil (spart Rechenzeit, aendert physikalisch
    praktisch nichts). Grenze: --min-collision-size;
  * einzelne Teile ueber MuJoCos Grenze werden erst groeber vernetzt und
    notfalls raeumlich in mehrere Meshes geteilt.

Ergebnis (Kommandozeile) - eine eigenstaendige Umgebung, siehe env_store.py:
  scenes/<name>/umgebung.xml                       fertige Umgebung (beim G1-Start waehlbar)
  scenes/<name>/meshes/<name>/*.stl                die Einzelteil-Meshes (binaer, Meter)
  scenes/<name>/meshes/<name>/cad_import.json      Manifest: Quelle, Optionen, Teile, Farben

Im Editor landet der Import erst in der Zwischenablage meshes/cad/<name>/ und
wird beim Speichern in den Ordner der Umgebung kopiert.

Das Manifest dient zugleich als Cache: wird dieselbe Datei mit denselben
Optionen noch einmal importiert, ist das Ergebnis sofort da. Und es sagt dem
Editor die Farben/Kollision der Teile (robits kennt dafuer keine Felder).

Unterstuetzte Formate
---------------------
  CAD  (OpenCascade, `cadquery-ocp`):  .step .stp .iges .igs .brep .brp
  Mesh (trimesh):  .stl .obj .ply .off .glb .gltf .3mf .dae
       -> ASCII-STL, zu grosse Netze, Szenen mit mehreren Objekten, Farben

Aufruf:
    python cad_import.py zelle.stp                    # -> scenes/zelle/umgebung.xml
    python cad_import.py zelle.stp --name demo --quality coarse
    python cad_import.py teil.step --place cad        # CAD-Koordinaten behalten
                                                      # (Default auto: G1 auf freien Platz)
    python cad_import.py --check                      # CAD-Backend vorhanden?

Nur Standardbibliothek beim Import des Moduls: numpy/OCP/trimesh werden erst in
den Funktionen geladen, die sie brauchen. So koennen auch build_env_scene.py
(blankes System-python3) und die Tests dieses Modul benutzen.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import env_store  # noqa: E402
import mesh_utils  # noqa: E402

MESHES_DIR = HERE / "meshes"
#: Zwischenablage fuer Importe im Editor: meshes/cad/<name>/. Beim Speichern
#: kopiert der Editor die benutzten Teile in den Ordner der Umgebung.
CAD_DIR = MESHES_DIR / "cad"
MANIFEST_NAME = "cad_import.json"

#: Wird hochgezaehlt, wenn sich das Ergebnis bei gleicher Eingabe aendert -
#: alte Manifest-Caches werden dann automatisch neu gebaut.
IMPORT_VERSION = 2

#: CAD-Formate (exakte Geometrie, brauchen OpenCascade)
CAD_SUFFIXES = {".step": "step", ".stp": "step", ".iges": "iges", ".igs": "iges",
                ".brep": "brep", ".brp": "brep"}
#: Mesh-Formate, die ueber trimesh eingelesen werden
MESH_SUFFIXES = (".stl", ".obj", ".ply", ".off", ".glb", ".gltf", ".3mf", ".dae")

#: Tesselierung: (lineare Abweichung relativ zur Teile-Diagonale,
#: Untergrenze m, Obergrenze m, Winkelabweichung rad). Die Grenzen sorgen
#: dafuer, dass eine Schraube nicht so fein wird wie eine Hallenwand und eine
#: 5-m-Wand nicht zentimeter-grob.
QUALITY = {
    "coarse": (0.004, 0.001, 0.010, 0.8),
    "normal": (0.002, 0.0005, 0.005, 0.5),
    "fine": (0.0007, 0.0002, 0.002, 0.3),
}
DEFAULT_QUALITY = "normal"

#: Teile, deren Bounding-Box-Diagonale darunter liegt, nur anzeigen (Meter).
DEFAULT_MIN_COLLISION_SIZE = 0.03

#: Konvexe Zerlegung (CoACD) fuer Teile, deren konvexe Huelle mehr als so viel
#: LEEREN Raum einschliesst (m^3) - z.B. eine L-foermige Wand, deren Huelle
#: sonst die halbe Zelle als massiven Block fuellt. Kleine/wenig konkave Teile
#: (Profile mit Nuten, Bleche) behalten ihre einfache Huelle.
DEFAULT_DECOMPOSE_MIN_EMPTY = 0.25
#: MuJoCo-Gruppe fuer reine Kollisions-Geoms (im Viewer standardmaessig aus).
COLLISION_GROUP = "3"

#: auto   = Boden auf z=0; begehbare Modelle (Zellen) so, dass der G1 (steht
#:          immer im Ursprung) auf einem freien Platz moeglichst mittig steht,
#:          kleine Objekte 1 m vor den G1
#: floor  = nur Boden auf z=0, XY aus dem CAD
#: center = Boden auf z=0, Mitte der Grundflaeche in den Ursprung
#: cad    = Koordinaten unveraendert
PLACEMENTS = ("auto", "floor", "center", "cad")
DEFAULT_PLACEMENT = "auto"

#: Platz, den der G1 im Ursprung braucht (Halbmasse x/y in m, Hoehe in m).
G1_FOOTPRINT_HALF = 0.3
G1_HEIGHT = 1.5

DEFAULT_RGBA = (0.7, 0.7, 0.72, 1.0)

#: Namen, die CAD-Systeme fuer unbenannte Koerper vergeben - dann wird der
#: Name der Baugruppe davorgesetzt, sonst heisst die halbe Szene "SOLID_17".
_GENERIC_NAMES = {"", "solid", "compound", "shell", "body", "part", "mesh",
                  "geometry", "open_shell", "closed_shell", "brep", "noname"}
_GENERIC_RE = re.compile(r"^(open_?cascade|unnamed|unbenannt|none$|null$)", re.I)


def _is_generic(name: str) -> bool:
    clean = sanitize_name(name, "")
    return clean.lower() in _GENERIC_NAMES or bool(_GENERIC_RE.match(clean))

#: Glas/Scheiben werden halbtransparent dargestellt (STEP liefert selten Alpha),
#: sonst sieht man im Viewer nicht in eine eingehauste Zelle hinein.
_GLASS_RE = re.compile(r"glas|scheibe|window|fenster|acryl|plexi|makrolon", re.I)
_GLASS_ALPHA = 0.35


def _log(msg: str) -> None:
    print(f"[cad_import] {msg}", flush=True)


class ImportFailed(RuntimeError):
    """Import nicht moeglich - Text ist fuer Menschen gedacht."""


# ---------------------------------------------------------------------------
# Namen
# ---------------------------------------------------------------------------
_UMLAUTS = {"ä": "ae", "ö": "oe", "ü": "ue", "ß": "ss", "Ä": "Ae", "Ö": "Oe", "Ü": "Ue"}


def sanitize_name(raw: str, fallback: str = "teil") -> str:
    """MuJoCo-/Dateisystem-taugliche Namen: A-Z a-z 0-9 _ - ."""
    name = str(raw or "")
    for k, v in _UMLAUTS.items():
        name = name.replace(k, v)
    name = re.sub(r"[^A-Za-z0-9_.\-]+", "_", name)
    name = re.sub(r"_{2,}", "_", name).strip("_.-")
    return name or fallback


def _unique(name: str, used: set) -> str:
    """Eindeutig machen (ohne Gross-/Kleinschreibung - Windows-Dateisysteme)."""
    cand, i = name, 2
    while cand.lower() in used:
        cand = f"{name}_{i}"
        i += 1
    used.add(cand.lower())
    return cand


def is_cad_file(path) -> bool:
    return Path(path).suffix.lower() in CAD_SUFFIXES


def is_importable(path) -> bool:
    suffix = Path(path).suffix.lower()
    return suffix in CAD_SUFFIXES or suffix in MESH_SUFFIXES


def supported_suffixes() -> list[str]:
    """Alle Endungen, die import_file() versteht (je nach installierten Backends)."""
    out = []
    if occ_available():
        out += list(CAD_SUFFIXES)
    if trimesh_available():
        out += list(MESH_SUFFIXES)
    else:
        out += list(mesh_utils.MJ_MESH_SUFFIXES)
    return sorted(set(out))


# ---------------------------------------------------------------------------
# Backends
# ---------------------------------------------------------------------------
def occ_available() -> bool:
    """OpenCascade-Bindings (cadquery-ocp) installiert? Ohne echten Import."""
    from importlib.util import find_spec
    try:
        return find_spec("OCP") is not None
    except (ImportError, ValueError):
        return False


def trimesh_available() -> bool:
    from importlib.util import find_spec
    try:
        return find_spec("trimesh") is not None and find_spec("numpy") is not None
    except (ImportError, ValueError):
        return False


def coacd_available() -> bool:
    """Konvexe Zerlegung (CoACD) installiert?"""
    from importlib.util import find_spec
    try:
        return find_spec("coacd") is not None and trimesh_available()
    except (ImportError, ValueError):
        return False


NO_OCC_HINT = (
    "Fuer CAD-Dateien (STEP/IGES/BREP) fehlt OpenCascade. Im Editor-venv "
    "nachinstallieren:\n"
    "    cd unitree_mujoco/scene_editor && .venv/bin/pip install cadquery-ocp\n"
    "(./launch.sh macht das beim naechsten Start automatisch). Alternativ die "
    "Datei im CAD-Programm als STL/OBJ exportieren.")


# ---------------------------------------------------------------------------
# Datenmodell
# ---------------------------------------------------------------------------
@dataclass
class ImportOptions:
    quality: str = DEFAULT_QUALITY
    #: None = automatisch (CAD: Einheit aus der Datei -> Meter; Meshes: 1.0)
    scale: float | None = None
    min_collision_size: float = DEFAULT_MIN_COLLISION_SIZE
    collision: bool = True
    #: konkave Teile in konvexe Stuecke zerlegen (braucht 'coacd'); Grenze in m^3
    #: leerer Huellen-Raum, ab der zerlegt wird. <= 0 schaltet ab.
    decompose_min_empty: float = DEFAULT_DECOMPOSE_MIN_EMPTY
    place: str = DEFAULT_PLACEMENT

    def cache_key(self) -> dict:
        """Optionen, die die erzeugten Meshes/Instanzen beeinflussen.
        (place wirkt nur auf die Umgebungs-XML, nicht auf die Meshes.)"""
        return {"quality": self.quality, "scale": self.scale,
                "min_collision_size": round(float(self.min_collision_size), 6),
                "collision": bool(self.collision),
                "decompose_min_empty": round(float(self.decompose_min_empty), 6),
                "decomposer": bool(coacd_available()),
                "version": IMPORT_VERSION}

    def validate(self) -> None:
        if self.quality not in QUALITY:
            raise ImportFailed(f"Unbekannte Genauigkeit {self.quality!r} "
                               f"(erlaubt: {', '.join(QUALITY)}).")
        if self.place not in PLACEMENTS:
            raise ImportFailed(f"Unbekannte Platzierung {self.place!r} "
                               f"(erlaubt: {', '.join(PLACEMENTS)}).")
        if self.scale is not None and not (self.scale > 0):
            raise ImportFailed("Skalierung muss > 0 sein.")


@dataclass
class MeshFile:
    """Eine geschriebene STL (zentriert auf ihre Bounding-Box-Mitte).

    Rollen: visual+collision (Normalfall), nur visual (kleine Teile oder
    zerlegte konkave Teile - deren Kollision uebernehmen die Huellen), nur
    collision (konvexe Stuecke einer Zerlegung, im Viewer ausgeblendet).
    """
    file: str                 # Dateiname innerhalb des Import-Ordners
    faces: int
    half: list                # halbe Kantenlaengen (m)
    rgba: list                # Standardfarbe (fuer den Editor)
    collision: bool = True
    visual: bool = True
    #: offenes/flaches Netz: MuJoCo kann kein Volumen berechnen ("mesh volume
    #: is too small") -> <mesh inertia="shell">
    shell: bool = False


@dataclass
class Instance:
    """Ein platziertes Teil = ein <geom> in der Umgebung."""
    name: str
    path: str                 # Baugruppen-Pfad "A/B/Teil" (nur zur Info)
    mesh: str                 # MeshFile.file
    pos: list                 # Meter, CAD-Koordinaten (ohne Platzierung)
    quat: list                # w x y z
    rgba: list
    collision: bool = True
    visual: bool = True
    part: str = ""            # Kollisions-Stueck: Name des sichtbaren Teils


@dataclass
class ImportResult:
    name: str
    source: str
    out_dir: Path
    meshes: dict = field(default_factory=dict)       # file -> MeshFile
    instances: list = field(default_factory=list)    # [Instance]
    bbox_min: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bbox_max: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    unit: float = 1.0
    notes: list = field(default_factory=list)
    from_cache: bool = False
    seconds: float = 0.0

    @property
    def parts(self) -> list:
        """Die sichtbaren Teile (ohne reine Kollisions-Stuecke)."""
        return [i for i in self.instances if i.visual]

    @property
    def total_faces(self) -> int:
        by_file = {m.file: m.faces for m in self.meshes.values()}
        return sum(by_file.get(i.mesh, 0) for i in self.parts)

    def world_boxes(self, collision_only: bool = False):
        """Achsenparallele Boxen (lo, hi) aller Teile in CAD-Koordinaten.

        Konservativ: die gedrehte Mesh-Box wird achsenparallel umschlossen.
        -> (lo (n,3), hi (n,3), [Instance])
        """
        import numpy as np

        insts = [i for i in self.instances
                 if (i.collision if collision_only else i.visual)]
        if not insts:
            return np.zeros((0, 3)), np.zeros((0, 3)), []
        half = np.array([self.meshes[i.mesh].half for i in insts], dtype=float)
        pos = np.array([i.pos for i in insts], dtype=float)
        w, x, y, z = np.array([i.quat for i in insts], dtype=float).T
        R = np.stack([
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
            np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
            np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
        ], axis=1)
        ext = np.einsum("nij,nj->ni", np.abs(R), half)
        return pos - ext, pos + ext, insts

    def ground_z(self) -> float:
        """Hoehe des Bodens im Modell (CAD-Koordinaten).

        Hat das Modell eine Bodenplatte (flach, >= 30 % der Grundflaeche),
        zaehlt deren Oberkante - so stehen Anlagen richtig, auch wenn Anker
        oder Fuesse im CAD unter den Hallenboden reichen. Sonst die Unterkante.
        """
        import numpy as np

        lo, hi, _ = self.world_boxes()
        if len(lo) == 0:
            return float(self.bbox_min[2])
        full = max((self.bbox_max[0] - self.bbox_min[0]) *
                   (self.bbox_max[1] - self.bbox_min[1]), 1e-9)
        area = (hi[:, 0] - lo[:, 0]) * (hi[:, 1] - lo[:, 1])
        flat = (hi[:, 2] - lo[:, 2] <= 0.05) & (area >= 0.3 * full)
        cand = np.flatnonzero(flat)
        if len(cand):
            return float(hi[cand[np.argmax(area[cand])], 2])
        return float(self.bbox_min[2])

    def spawn_conflicts(self, offset) -> list:
        """Namen der Teile, in denen der G1 (Ursprung) nach Verschiebung steht."""
        import numpy as np

        lo, hi, insts = self.world_boxes(collision_only=True)
        if not insts:
            return []
        o = np.asarray(offset, dtype=float)
        r = G1_FOOTPRINT_HALF
        box_lo = np.array([-r, -r, 0.03]) - o
        box_hi = np.array([r, r, G1_HEIGHT]) - o
        hit = np.all((lo < box_hi) & (hi > box_lo), axis=1)
        return sorted({insts[k].part or insts[k].name for k in np.flatnonzero(hit)})

    def _free_spot(self, ground: float):
        """Freier Platz fuer den G1, moeglichst nah an der Mitte. -> (x, y) | None"""
        import numpy as np

        lo, hi, _ = self.world_boxes(collision_only=True)
        r = G1_FOOTPRINT_HALF
        keep = (hi[:, 2] > ground + 0.03) & (lo[:, 2] < ground + G1_HEIGHT)
        lo, hi = lo[keep, :2], hi[keep, :2]
        bx0, by0 = self.bbox_min[0] + r, self.bbox_min[1] + r
        bx1, by1 = self.bbox_max[0] - r, self.bbox_max[1] - r
        if bx1 < bx0 or by1 < by0:
            return None
        xs = np.arange(bx0, bx1 + 1e-9, 0.1)
        ys = np.arange(by0, by1 + 1e-9, 0.1)
        P = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        free = np.ones(len(P), dtype=bool)
        for k in range(0, len(lo), 256):                    # speicherschonend
            l, h = lo[k:k + 256], hi[k:k + 256]
            inside = np.all((l[None] < P[:, None] + r) & (h[None] > P[:, None] - r), axis=2)
            free &= ~inside.any(axis=1)
        if not free.any():
            return None
        center = np.array([(self.bbox_min[0] + self.bbox_max[0]) / 2,
                           (self.bbox_min[1] + self.bbox_max[1]) / 2])
        cand = P[free]
        best = cand[np.argmin(np.linalg.norm(cand - center, axis=1))]
        return float(best[0]), float(best[1])

    def placement_offset(self, place: str) -> list:
        """Verschiebung der ganzen Baugruppe fuer die gewaehlte Platzierung.

        Der G1 steht immer im Ursprung - also wird die Umgebung verschoben.
        """
        if place == "cad":
            return [0.0, 0.0, 0.0]
        lo, hi = self.bbox_min, self.bbox_max
        dz = -self.ground_z()
        if place == "floor":
            return [0.0, 0.0, dz]
        cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
        if place == "center":
            return [-cx, -cy, dz]
        # auto: begehbar (Grundflaeche deutlich groesser als der G1)?
        walkable = min(hi[0] - lo[0], hi[1] - lo[1]) > 4 * G1_FOOTPRINT_HALF + 0.4
        if walkable:
            spot = self._free_spot(-dz)
            if spot is not None:
                return [-spot[0], -spot[1], dz]
        # Klein (oder kein Platz drin): 1 m vor den G1 (+x), mittig.
        return [1.0 - lo[0], -cy, dz]

    def summary(self) -> str:
        size = [h - lo for lo, h in zip(self.bbox_min, self.bbox_max)]
        colliding = {i.part or i.name for i in self.instances if i.collision}
        n_split = len({i.part for i in self.instances if i.part})
        split = f", {n_split} davon konvex zerlegt" if n_split else ""
        return (f"{len(self.parts)} Teile ({len(colliding)} mit Kollision{split}), "
                f"{sum(1 for m in self.meshes.values() if m.visual)} verschiedene Meshes, "
                f"{mesh_utils._de_number(self.total_faces)} Dreiecke, "
                f"Groesse {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m")


# ---------------------------------------------------------------------------
# Kleine Mathe-Helfer (numpy nur innerhalb der Funktionen)
# ---------------------------------------------------------------------------
def _srgb(c: float) -> float:
    """Lineares RGB (OpenCascade intern) -> sRGB (was CAD anzeigt)."""
    c = min(max(float(c), 0.0), 1.0)
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1.0 / 2.4) - 0.055


def _mat_to_quat(R) -> list:
    """Rotationsmatrix -> Quaternion (w, x, y, z), numerisch stabil."""
    m00, m01, m02 = float(R[0][0]), float(R[0][1]), float(R[0][2])
    m10, m11, m12 = float(R[1][0]), float(R[1][1]), float(R[1][2])
    m20, m21, m22 = float(R[2][0]), float(R[2][1]), float(R[2][2])
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    q = [v / n for v in q]
    if q[0] < 0:                                   # eindeutige Darstellung
        q = [-v for v in q]
    return q


def _split_rigid(M):
    """4x4-Matrix -> (R, t, S) mit M = [R*S | t], R echte Drehung.

    S ist der Anteil, den eine Quaternion nicht darstellen kann
    (Skalierung, Spiegelung) und der deshalb ins Mesh eingebacken wird.
    """
    import numpy as np

    L = np.asarray(M, dtype=float)[:3, :3]
    t = np.asarray(M, dtype=float)[:3, 3]
    det = float(np.linalg.det(L))
    if abs(det) < 1e-12:
        raise ValueError("entartete Transformation")
    s = abs(det) ** (1.0 / 3.0)
    F = np.diag([1.0, 1.0, -1.0]) if det < 0 else np.eye(3)
    R = (L / s) @ F
    # Numerisches Rauschen aus verketteten CAD-Transformationen wegputzen.
    u, _sv, vt = np.linalg.svd(R)
    R = u @ vt
    return R, t, s * F


def split_faces(vertices, faces, max_faces: int):
    """Netz raeumlich in Stuecke mit hoechstens max_faces Dreiecken teilen.

    Median-Schnitt entlang der laengsten Achse der Dreiecks-Schwerpunkte,
    rekursiv. Liefert Liste von (vertices, faces) mit kompakten Indizes.
    """
    import numpy as np

    faces = np.asarray(faces)
    vertices = np.asarray(vertices)
    if len(faces) <= max_faces:
        return [(vertices, faces)]
    out = []
    stack = [np.arange(len(faces))]
    centroids = vertices[faces].mean(axis=1)
    while stack:
        idx = stack.pop()
        if len(idx) <= max_faces:
            sub = faces[idx]
            used, inv = np.unique(sub.ravel(), return_inverse=True)
            out.append((vertices[used], inv.reshape(-1, 3)))
            continue
        c = centroids[idx]
        axis = int(np.argmax(c.max(axis=0) - c.min(axis=0)))
        order = np.argsort(c[:, axis], kind="stable")
        half = len(idx) // 2
        stack.append(idx[order[:half]])
        stack.append(idx[order[half:]])
    return out


def mesh_is_solid(vertices, faces, min_volume: float = 1e-9) -> bool:
    """Geschlossenes Netz mit echtem Volumen? (sonst braucht MuJoCo 'shell').

    Geschlossen = jede Kante gehoert zu genau zwei Dreiecken. Punkte werden
    vorher auf 0.1 um zusammengefasst, weil CAD-Netze jede Flaeche mit eigenen
    Knoten liefern.
    """
    import numpy as np

    v = np.asarray(vertices, dtype=float)
    f = np.asarray(faces, dtype=np.int64)
    if len(f) < 4:
        return False
    _, remap = np.unique(np.round(v / 1e-7).astype(np.int64), axis=0, return_inverse=True)
    f = remap.reshape(-1)[f]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f) < 4:
        return False
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    if not np.all(counts == 2):
        return False
    tri = v[np.asarray(faces, dtype=np.int64)]
    volume = np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6.0
    return abs(float(volume)) > min_volume


def write_stl(dest: Path, vertices, faces) -> int:
    """Binaere STL mit numpy schreiben (schnell auch fuer grosse Netze)."""
    import numpy as np

    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)
    tri = v[f]                                            # (n, 3, 3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1)
    length[length == 0] = 1.0
    n = n / length[:, None]
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = tri
    with open(dest, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())
    return len(f)


# ---------------------------------------------------------------------------
# Zwischenformat der Leser
# ---------------------------------------------------------------------------
@dataclass
class _Proto:
    """Ein Bauteil-Koerper in seinem eigenen Koordinatensystem (Quell-Einheit)."""
    key: str
    name: str
    vertices: object            # np.ndarray (n, 3)
    faces: object               # np.ndarray (m, 3)
    rgba: tuple | None = None


@dataclass
class _RawInstance:
    proto: str
    name: str
    path: str
    matrix: object              # 4x4 (Quell-Einheit), darf spiegeln/skalieren
    rgba: tuple | None = None


# ---------------------------------------------------------------------------
# Leser 1: OpenCascade (STEP / IGES / BREP) mit Baugruppenstruktur
# ---------------------------------------------------------------------------
def _occ_label_seq():
    try:                                   # OCP 7.x
        from OCP.TDF import TDF_LabelSequence
        return TDF_LabelSequence()
    except ImportError:                    # OCP 8.x: Sammlungen umgezogen
        from OCP.OCP.collections import Sequence_TDF_Label
        return Sequence_TDF_Label()


def _occ_shape_map():
    try:                                   # OCP 7.x
        from OCP.TopTools import TopTools_IndexedMapOfShape
        return TopTools_IndexedMapOfShape()
    except ImportError:                    # OCP 8.x
        from OCP.OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher
        return IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher()


def _occ_bbox(shape):
    """(min, max) einer Form - funktioniert mit OCP 7 und 8 (Bnd_Box.Get()
    liefert in OCP 8 ein nicht nach Python wandelbares Struct)."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        return None
    lo, hi = box.CornerMin(), box.CornerMax()
    return (lo.X(), lo.Y(), lo.Z()), (hi.X(), hi.Y(), hi.Z())


def _occ_trsf_matrix(trsf):
    import numpy as np

    M = np.eye(4)
    for r in range(3):
        for c in range(4):
            M[r, c] = trsf.Value(r + 1, c + 1)
    return M


def _occ_unit_to_m(doc) -> float:
    """Laengeneinheit des eingelesenen Dokuments in Metern (mm -> 0.001).

    OpenCascade rechnet alles in eine Dokument-Einheit um (Standard mm), egal
    ob die Datei in mm, inch oder m vorliegt. Der Wert laesst sich je nach
    Bindings-Version nicht immer abfragen - dann gilt OCCTs Standard (mm).
    """
    try:
        from OCP.XCAFDoc import XCAFDoc_DocumentTool
        res = XCAFDoc_DocumentTool.GetLengthUnit_s(doc)
        if isinstance(res, tuple) and len(res) == 2 and res[0] and res[1] > 0:
            return float(res[1])
        if isinstance(res, float) and res > 0:
            return res
    except Exception:
        pass
    try:
        from OCP.Interface import Interface_Static
        unit = (Interface_Static.CVal_s("xstep.cascade.unit") or "MM").upper()
        return {"MM": 0.001, "M": 1.0, "CM": 0.01, "IN": 0.0254,
                "INCH": 0.0254, "FT": 0.3048, "UM": 1e-6, "KM": 1000.0}.get(unit, 0.001)
    except Exception:
        return 0.001


def _occ_read_document(src: Path, kind: str):
    """Datei in ein XCAF-Dokument lesen (mit Namen/Farben/Struktur)."""
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document

    doc = TDocStd_Document(TCollection_ExtendedString("cad_import"))
    if kind == "step":
        from OCP.STEPCAFControl import STEPCAFControl_Reader as Reader
    else:
        from OCP.IGESCAFControl import IGESCAFControl_Reader as Reader
    reader = Reader()
    for setter in ("SetNameMode", "SetColorMode", "SetLayerMode"):
        if hasattr(reader, setter):
            getattr(reader, setter)(True)
    status = reader.ReadFile(str(src))
    if "RetDone" not in str(status) and status != 1:
        raise ImportFailed(
            f"{src.name} laesst sich nicht lesen ({status}). Ist das wirklich "
            f"eine {kind.upper()}-Datei? Ggf. im CAD neu exportieren (AP214/AP242).")
    if not reader.Transfer(doc):
        raise ImportFailed(f"{src.name}: Geometrie konnte nicht uebernommen werden.")
    return doc


class _OccReader:
    """Laeuft den XCAF-Baugruppenbaum ab und tesseliert jedes Bauteil einmal."""

    def __init__(self, quality: str, unit: float, log, root_name: str = "teil"):
        self.rel_lin, self.min_abs, self.max_abs, self.ang = QUALITY[quality]
        self.unit = unit              # Quell-Einheit in Metern
        self.log = log
        self.root_name = root_name    # Name fuer unbenannte Teile ohne Baugruppe
        self.protos: dict = {}        # key -> [_Proto]
        self.instances: list = []
        self.notes: list = []
        self._label_pieces: dict = {}  # label-entry -> [proto keys]
        self._t0 = time.time()
        self._n_meshed = 0

    # -- Namen / Farben --------------------------------------------------
    @staticmethod
    def _name(label) -> str:
        from OCP.TDataStd import TDataStd_Name

        attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
            try:
                return attr.Get().ToExtString()
            except Exception:
                return ""
        return ""

    @staticmethod
    def _entry(label) -> str:
        from OCP.TCollection import TCollection_AsciiString
        from OCP.TDF import TDF_Tool

        s = TCollection_AsciiString()
        TDF_Tool.Entry_s(label, s)
        return s.ToCString()

    def _color(self, label):
        """Farbe eines Labels (Flaechen- vor Allgemeinfarbe) als sRGB-rgba."""
        from OCP.Quantity import Quantity_ColorRGBA
        from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType

        col = Quantity_ColorRGBA()
        for ctype in (XCAFDoc_ColorType.XCAFDoc_ColorSurf,
                      XCAFDoc_ColorType.XCAFDoc_ColorGen):
            try:
                ok = XCAFDoc_ColorTool.GetColor_s(label, ctype, col)
            except Exception:
                ok = False
            if ok:
                rgb = col.GetRGB()
                return (_srgb(rgb.Red()), _srgb(rgb.Green()), _srgb(rgb.Blue()),
                        float(col.Alpha()))
        return None

    def _leaf_color(self, label):
        """Farbe eines Bauteils: am Teil selbst, sonst an seinen Unterformen
        (manche Systeme faerben nur die Flaechen/Koerper)."""
        from OCP.XCAFDoc import XCAFDoc_ShapeTool

        c = self._color(label)
        if c is not None:
            return c
        subs = _occ_label_seq()
        try:
            XCAFDoc_ShapeTool.GetSubShapes_s(label, subs)
        except Exception:
            return None
        for i in range(1, subs.Length() + 1):
            c = self._color(subs.Value(i))
            if c is not None:
                return c
        return None

    # -- Baum ------------------------------------------------------------
    def walk_document(self, doc) -> None:
        import numpy as np
        from OCP.XCAFDoc import XCAFDoc_DocumentTool

        shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        free = _occ_label_seq()
        shape_tool.GetFreeShapes(free)
        if free.Length() == 0:
            raise ImportFailed("Die Datei enthaelt keine Geometrie.")
        for i in range(1, free.Length() + 1):
            lab = free.Value(i)
            self._walk(lab, np.eye(4), [], None, None)

    def _walk(self, label, M, path, instance_rgba, inherited_rgba, name=None, depth=0):
        """instance_rgba: Farbe an der Instanz selbst (ueberschreibt alles),
        inherited_rgba: Farbe einer Baugruppe (nur fuer Teile ohne eigene)."""
        from OCP.TDF import TDF_Label
        from OCP.XCAFDoc import XCAFDoc_ShapeTool

        if depth > 64:
            self.notes.append("Baugruppe tiefer als 64 Ebenen - Rest ignoriert.")
            return
        name = name or self._name(label)
        if XCAFDoc_ShapeTool.IsAssembly_s(label):
            comps = _occ_label_seq()
            XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
            color = instance_rgba or self._color(label) or inherited_rgba
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                if not XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref):
                    continue
                loc = XCAFDoc_ShapeTool.GetLocation_s(comp)
                Mc = M @ _occ_trsf_matrix(loc.Transformation())
                cname = self._name(ref) or self._name(comp)
                self._walk(ref, Mc, path + [name] if name else path,
                           self._color(comp), color, name=cname, depth=depth + 1)
            return
        if XCAFDoc_ShapeTool.IsReference_s(label):
            ref = TDF_Label()
            if XCAFDoc_ShapeTool.GetReferredShape_s(label, ref):
                self._walk(ref, M, path, instance_rgba, inherited_rgba, name, depth + 1)
            return
        self._leaf(label, M, path, name, instance_rgba, inherited_rgba)

    def _leaf(self, label, M, path, name, instance_rgba, inherited_rgba):
        from OCP.TopLoc import TopLoc_Location
        from OCP.XCAFDoc import XCAFDoc_ShapeTool

        shape = XCAFDoc_ShapeTool.GetShape_s(label)
        if shape is None or shape.IsNull():
            return
        entry = self._entry(label)
        # Eine eigene Lage der Form (kommt bei Top-Level-Teilen vor) in die
        # Instanz-Matrix ziehen, damit das Bauteil selbst im Ursprung liegt.
        loc = shape.Location()
        if not loc.IsIdentity():
            M = M @ _occ_trsf_matrix(loc.Transformation())
            shape = shape.Located(TopLoc_Location())

        leaf_color = self._leaf_color(label)
        rgba = instance_rgba or leaf_color or inherited_rgba
        if entry not in self._label_pieces:
            self._label_pieces[entry] = self._mesh_leaf(entry, shape, name, leaf_color)
        pieces = self._label_pieces[entry]
        if not pieces:
            return
        clean = name
        if _is_generic(name):
            # "SOLID" -> "Schrank_SOLID"; "Open CASCADE STEP translator" -> Dateiname
            parent = next((p for p in reversed(path) if not _is_generic(p)), "")
            own = "" if _GENERIC_RE.match(sanitize_name(name, "")) else sanitize_name(name, "")
            clean = "_".join(x for x in (parent or self.root_name, own) if x)
        for key in pieces:
            self.instances.append(_RawInstance(
                proto=key, name=clean, path="/".join(path + [name or "teil"]),
                matrix=M, rgba=rgba))

    # -- Tesselierung ----------------------------------------------------
    def _deflection(self, shape, factor=1.0):
        bb = _occ_bbox(shape)
        if bb is None:
            return None
        diag = math.dist(bb[0], bb[1])
        lin_m = min(max(diag * self.unit * self.rel_lin, self.min_abs), self.max_abs)
        lin = lin_m * factor / self.unit
        ang = min(self.ang * math.sqrt(factor), 1.2)
        return max(lin, 1e-9), ang

    def _mesh_leaf(self, entry, shape, name, rgba) -> list:
        """Bauteil tesselieren und in Koerper aufteilen. -> Proto-Keys."""
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.BRepTools import BRepTools

        pieces = self._pieces(shape)
        if not pieces:
            self.notes.append(f"'{name or entry}' enthaelt keine Flaechen - uebersprungen.")
            return []
        keys = []
        factor = 1.0
        for attempt in range(4):
            defl = self._deflection(shape, factor)
            if defl is None:
                return []
            try:
                BRepMesh_IncrementalMesh(shape, defl[0], False, defl[1], True)
            except Exception as exc:
                self.notes.append(f"'{name}': Vernetzung fehlgeschlagen ({exc}) - uebersprungen.")
                return []
            arrays = [self._triangles(p) for p in pieces]
            biggest = max((len(a[1]) for a in arrays if a is not None), default=0)
            if biggest <= mesh_utils.MJ_MAX_FACES or attempt == 3:
                break
            # Zu fein fuer MuJoCo: groeber vernetzen (Dreiecke ~ 1/Abweichung^2).
            factor *= max(1.5, min(math.sqrt(biggest / mesh_utils.MJ_MAX_FACES) * 1.3, 8.0))
            BRepTools.Clean_s(shape)
        if factor > 1.0:
            self.notes.append(f"'{name}': sehr fein - Vernetzung um Faktor {factor:.1f} "
                              "vergroebert.")
        for i, arr in enumerate(arrays):
            if arr is None or len(arr[1]) == 0:
                continue
            key = f"{entry}#{i}"
            self.protos[key] = _Proto(key=key, name=name, vertices=arr[0],
                                      faces=arr[1], rgba=rgba)
            keys.append(key)
        self._n_meshed += 1
        if self._n_meshed % 50 == 0:
            self.log(f"  {self._n_meshed} Bauteile vernetzt ({time.time() - self._t0:.0f} s) ...")
        return keys

    @staticmethod
    def _pieces(shape) -> list:
        """Form in Volumenkoerper zerlegen (+ lose Flaechen als eigenes Stueck).

        Grund: MuJoCo kollidiert ueber die konvexe Huelle je Mesh - zwei
        getrennte Koerper in einem Mesh wuerden zu einem Klotz verschmelzen.
        """
        from OCP.BRep import BRep_Builder
        from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
        from OCP.TopExp import TopExp, TopExp_Explorer
        from OCP.TopoDS import TopoDS_Compound

        solids = []
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            solids.append(exp.Current())
            exp.Next()
        n_faces = 0
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            n_faces += 1
            exp.Next()
        if n_faces == 0:
            return []
        if len(solids) <= 1:
            return [shape]

        in_solids = _occ_shape_map()
        for s in solids:
            TopExp.MapShapes_s(s, TopAbs_FACE, in_solids)
        loose = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(loose)
        n_loose = 0
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            if not in_solids.Contains(exp.Current()):
                builder.Add(loose, exp.Current())
                n_loose += 1
            exp.Next()
        return solids + ([loose] if n_loose else [])

    @staticmethod
    def _triangles(shape):
        """Triangulation einer (bereits vernetzten) Form -> (vertices, faces)."""
        import numpy as np
        from OCP.BRep import BRep_Tool
        from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS

        # OCP 7: Klasse TopoDS mit Face_s(), OCP 8: Modul-Funktion Face().
        to_face = getattr(TopoDS, "Face_s", None) or getattr(TopoDS, "Face")
        verts, faces, offset = [], [], 0
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            face = to_face(exp.Current())
            exp.Next()
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is None or tri.NbTriangles() == 0:
                continue
            n = tri.NbNodes()
            pts = np.empty((n, 3))
            for i in range(n):
                p = tri.Node(i + 1)
                pts[i] = (p.X(), p.Y(), p.Z())
            if not loc.IsIdentity():
                T = _occ_trsf_matrix(loc.Transformation())
                pts = pts @ T[:3, :3].T + T[:3, 3]
            m = tri.NbTriangles()
            idx = np.empty((m, 3), dtype=np.int64)
            for i in range(m):
                t = tri.Triangle(i + 1)
                idx[i] = (t.Value(1), t.Value(2), t.Value(3))
            idx -= 1
            if face.Orientation() == TopAbs_REVERSED:
                idx = idx[:, [0, 2, 1]]
            verts.append(pts)
            faces.append(idx + offset)
            offset += n
        if not faces:
            return None
        return np.vstack(verts), np.vstack(faces)


def _read_occ(src: Path, kind: str, options: ImportOptions, log):
    if not occ_available():
        raise ImportFailed(NO_OCC_HINT)
    t0 = time.time()
    log(f"{src.name} ({src.stat().st_size / 1e6:.1f} MB) wird eingelesen ...")
    if kind == "brep":
        from OCP.BRep import BRep_Builder
        from OCP.BRepTools import BRepTools
        from OCP.TopoDS import TopoDS_Shape

        shape = TopoDS_Shape()
        if not BRepTools.Read_s(shape, str(src), BRep_Builder()):
            raise ImportFailed(f"{src.name} ist keine lesbare BREP-Datei.")
        unit = 0.001                     # BREP hat keine Einheit - CAD-ueblich mm
        reader = _OccReader(options.quality, unit, log, sanitize_name(src.stem))
        import numpy as np
        keys = reader._mesh_leaf("brep", shape, src.stem, None)
        for key in keys:
            reader.instances.append(_RawInstance(proto=key, name=src.stem,
                                                 path=src.stem, matrix=np.eye(4)))
        reader.notes.append("BREP-Dateien haben keine Einheit - mm angenommen "
                            "(sonst --scale angeben).")
    else:
        doc = _occ_read_document(src, kind)
        unit = _occ_unit_to_m(doc)
        log(f"  gelesen in {time.time() - t0:.0f} s, vernetze Bauteile ...")
        reader = _OccReader(options.quality, unit, log, sanitize_name(src.stem))
        reader.walk_document(doc)
    log(f"  {len(reader.instances)} Instanzen, {len(reader.protos)} verschiedene "
        f"Koerper ({time.time() - t0:.0f} s).")
    return reader.protos, reader.instances, unit, reader.notes


# ---------------------------------------------------------------------------
# Leser 2: trimesh (STL/OBJ/PLY/GLB/...)
# ---------------------------------------------------------------------------
def _trimesh_rgba(mesh):
    try:
        vis = mesh.visual
        if getattr(vis, "kind", None) == "texture":
            mat = vis.material
            c = getattr(mat, "baseColorFactor", None)
            if c is None:
                c = getattr(mat, "main_color", None)
        elif getattr(vis, "defined", False):
            c = vis.main_color
        else:
            return None
        if c is None:
            return None
        c = [float(x) for x in c]
        if len(c) == 3:
            c.append(255.0 if max(c) > 1.0 else 1.0)
        if max(c) > 1.0:
            c = [x / 255.0 for x in c]
        return tuple(c[:4])
    except Exception:
        return None


def _read_trimesh(src: Path, options: ImportOptions, log):
    if not trimesh_available():
        raise ImportFailed(
            f"{src.name}: fuer dieses Format wird 'trimesh' gebraucht "
            "(.venv/bin/pip install trimesh).")
    import numpy as np
    import trimesh

    log(f"{src.name} ({src.stat().st_size / 1e6:.1f} MB) wird eingelesen ...")
    try:
        scene = trimesh.load(str(src), force="scene", process=False)
    except Exception as exc:
        raise ImportFailed(f"{src.name} laesst sich nicht lesen: {exc}") from exc

    protos, instances = {}, []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph.get(node)
        geom = scene.geometry.get(gname)
        if not isinstance(geom, trimesh.Trimesh) or len(geom.faces) == 0:
            continue
        if gname not in protos:
            protos[gname] = _Proto(key=gname, name=gname,
                                   vertices=np.asarray(geom.vertices, dtype=float),
                                   faces=np.asarray(geom.faces, dtype=np.int64),
                                   rgba=_trimesh_rgba(geom))
        name = node if node != gname or len(scene.graph.nodes_geometry) > 1 else src.stem
        instances.append(_RawInstance(proto=gname, name=sanitize_name(name, src.stem),
                                      path=str(node), matrix=np.asarray(T, dtype=float)))
    if not instances:
        raise ImportFailed(f"{src.name} enthaelt keine Dreiecksnetze.")
    return protos, instances, 1.0, []


# ---------------------------------------------------------------------------
# Gemeinsamer Schreiber
# ---------------------------------------------------------------------------
def _decompose(vertices, faces, min_empty: float):
    """Konvexe Zerlegung, falls die Huelle viel leeren Raum einschliesst.

    -> (status, stuecke):  ("skip", None)  nicht noetig/nicht bestimmbar,
                           ("missing", leer_m3)  noetig, aber coacd fehlt,
                           ("ok", [(v, f), ...])
    """
    if min_empty <= 0 or not trimesh_available():
        return "skip", None
    import numpy as np
    import trimesh

    mesh = trimesh.Trimesh(vertices, faces, process=True)
    if not mesh.is_watertight:
        return "skip", None                 # Volumen unbestimmt -> Huelle behalten
    try:
        hull = float(mesh.convex_hull.volume)
    except Exception:
        return "skip", None
    vol = abs(float(mesh.volume))
    empty = hull - vol
    if empty < min_empty or vol > 0.5 * hull:
        return "skip", None
    if not coacd_available():
        return "missing", empty
    import coacd

    coacd.set_log_level("error")
    kwargs = dict(threshold=0.05, max_convex_hull=32, preprocess_mode="auto",
                  resolution=2000, mcts_iterations=100, mcts_max_depth=2, seed=0)
    try:
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **kwargs)
    except TypeError:                       # aeltere coacd ohne seed
        kwargs.pop("seed")
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **kwargs)
    except Exception:
        return "skip", None
    parts = [(np.asarray(v, dtype=float), np.asarray(f, dtype=np.int64))
             for v, f in parts if len(f) >= 4]
    if len(parts) <= 1:
        return "skip", None
    return "ok", parts


def _build(protos, raw_instances, unit, options: ImportOptions, out_dir: Path,
           result: ImportResult, log) -> None:
    """Meshes schreiben und Instanzen in Meter/Quaternionen umrechnen."""
    import numpy as np

    scale = unit * (options.scale if options.scale is not None else 1.0)
    variants = {}           # (proto, S-Schluessel) -> [Stueck-Dict]
    files = {}              # (Geometrie-Hash, Farbe) -> file   (gleiche Teile nur 1x)
    decomp = {}             # Geometrie-Hash -> (status, stuecke)   (CoACD nur 1x)
    hulls = {}              # file -> [(hull_file, center im Teil-Frame)]
    missing_decomp = []     # [(name, leer_m3)] wenn coacd fehlt
    used_files: set = set()
    used_names: set = set()
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)

    def new_file(base: str, verts, faces, rgba, collision, visual) -> str:
        file = _unique(base, used_files) + ".stl"
        n = write_stl(out_dir / file, verts, faces)
        half = (verts.max(axis=0) - verts.min(axis=0)) / 2.0
        result.meshes[file] = MeshFile(file=file, faces=n, half=[float(x) for x in half],
                                       rgba=[round(float(x), 4) for x in rgba],
                                       collision=collision, visual=visual,
                                       shell=not mesh_is_solid(verts, faces))
        return file

    for raw in raw_instances:
        proto = protos.get(raw.proto)
        if proto is None:
            continue
        try:
            R, t, S = _split_rigid(raw.matrix)
        except ValueError:
            result.notes.append(f"'{raw.name}': entartete Lage - uebersprungen.")
            continue
        vkey = (raw.proto, tuple(np.round(S, 6).ravel()))
        if vkey not in variants:
            V = (np.asarray(proto.vertices, dtype=float) @ S.T) * scale
            F = np.asarray(proto.faces, dtype=np.int64)
            if np.linalg.det(S) < 0:
                F = F[:, [0, 2, 1]]                  # Spiegeln dreht die Umlaufrichtung
            chunks = split_faces(V, F, mesh_utils.MJ_MAX_FACES)
            if len(chunks) > 1:
                result.notes.append(f"'{proto.name}': {mesh_utils._de_number(len(F))} "
                                    f"Dreiecke - in {len(chunks)} Meshes geteilt "
                                    "(MuJoCo-Grenze).")
            pieces = []
            for cv, cf in chunks:
                cmin, cmax = cv.min(axis=0), cv.max(axis=0)
                center = (cmin + cmax) / 2.0
                cv = cv - center
                pieces.append({
                    "center": center, "verts": cv, "faces": cf,
                    "digest": hashlib.sha1(np.round(cv, 6).tobytes() + cf.tobytes()).hexdigest(),
                    "size": 2.0 * float(np.linalg.norm((cmax - cmin) / 2.0))})
            variants[vkey] = pieces

        # Farbe gehoert zur Datei: der Editor kennt Farben nur je Mesh-Datei
        # (Manifest). Gleiche Geometrie in zwei Farben = zwei Dateien, sonst
        # waeren nach einem Editor-Durchlauf beide gleich eingefaerbt.
        rgba = [float(x) for x in (raw.rgba or proto.rgba or DEFAULT_RGBA)]
        if _GLASS_RE.search(raw.name) or _GLASS_RE.search(proto.name):
            rgba[3] = min(rgba[3], _GLASS_ALPHA)
        rgba = [round(x, 4) for x in rgba]
        quat = [round(float(x), 8) for x in _mat_to_quat(R)]
        for piece in variants[vkey]:
            cv, cf = piece["verts"], piece["faces"]
            fkey = (piece["digest"], tuple(round(x, 3) for x in rgba))
            file = files.get(fkey)
            if file is None:
                collide = options.collision and piece["size"] >= options.min_collision_size
                status, parts = ("skip", None)
                if collide:
                    if piece["digest"] not in decomp:
                        decomp[piece["digest"]] = _decompose(cv, cf, options.decompose_min_empty)
                        st, pp = decomp[piece["digest"]]
                        if st == "missing":
                            missing_decomp.append((proto.name, pp))
                        elif st == "ok":
                            log(f"  '{proto.name}' ist stark konkav -> in {len(pp)} "
                                "konvexe Stuecke zerlegt.")
                    status, parts = decomp[piece["digest"]]
                base = sanitize_name(proto.name, "teil")[:60]
                file = new_file(base, cv, cf, rgba, collide and status != "ok", True)
                if status == "ok":
                    # Kollision uebernehmen die konvexen Stuecke; das
                    # sichtbare Mesh selbst kollidiert nicht mehr.
                    hulls[file] = []
                    for hv, hf in parts:
                        hc = (hv.min(axis=0) + hv.max(axis=0)) / 2.0
                        hfile = new_file(f"{file[:-4]}__huelle", hv - hc, hf, rgba,
                                         True, False)
                        hulls[file].append((hfile, hc))
                files[fkey] = file
            pos = R @ piece["center"] + t * scale
            world = cv @ R.T + pos
            lo = np.minimum(lo, world.min(axis=0))
            hi = np.maximum(hi, world.max(axis=0))
            name = _unique(sanitize_name(raw.name, "teil"), used_names)
            result.instances.append(Instance(
                name=name, path=raw.path, mesh=file,
                pos=[round(float(x), 6) for x in pos], quat=quat, rgba=rgba,
                collision=result.meshes[file].collision))
            for k, (hfile, hc) in enumerate(hulls.get(file, ()), start=1):
                hpos = R @ (piece["center"] + hc) + t * scale
                result.instances.append(Instance(
                    name=_unique(f"{name}__k{k}", used_names), path=raw.path,
                    mesh=hfile, pos=[round(float(x), 6) for x in hpos], quat=quat,
                    rgba=rgba, collision=True, visual=False, part=name))
    if not result.instances:
        raise ImportFailed("Keine verwertbare Geometrie gefunden.")
    result.bbox_min = [round(float(x), 6) for x in lo]
    result.bbox_max = [round(float(x), 6) for x in hi]
    if missing_decomp:
        names = ", ".join(f"{n} ({e:.1f} m^3 leer)" for n, e in missing_decomp[:5])
        more = f" und {len(missing_decomp) - 5} weitere" if len(missing_decomp) > 5 else ""
        result.notes.append(
            f"Stark konkave Teile kollidieren als gefuellter Block (konvexe Huelle): "
            f"{names}{more}. Fuer genaue Kollision 'coacd' installieren "
            "(.venv/bin/pip install coacd) und neu importieren.")


# ---------------------------------------------------------------------------
# Manifest (Cache + Metadaten fuer Editor/build_env_scene)
# ---------------------------------------------------------------------------
def _source_stamp(src: Path) -> dict:
    st = src.stat()
    return {"path": str(src), "size": st.st_size, "mtime": int(st.st_mtime)}


def _write_manifest(result: ImportResult, options: ImportOptions, src: Path) -> None:
    data = {
        "version": IMPORT_VERSION,
        "name": result.name,
        "source": _source_stamp(src),
        "options": options.cache_key(),
        "unit": result.unit,
        "bbox_min": result.bbox_min,
        "bbox_max": result.bbox_max,
        "notes": result.notes,
        "meshes": {k: asdict(v) for k, v in result.meshes.items()},
        "instances": [asdict(i) for i in result.instances],
    }
    (result.out_dir / MANIFEST_NAME).write_text(json.dumps(data, indent=1), encoding="utf-8")


def load_manifest(out_dir: Path) -> ImportResult | None:
    """Gespeichertes Import-Ergebnis laden (None, wenn keins/kaputt)."""
    path = Path(out_dir) / MANIFEST_NAME
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        res = ImportResult(name=data["name"], source=data["source"]["path"],
                           out_dir=Path(out_dir), unit=data.get("unit", 1.0),
                           bbox_min=data["bbox_min"], bbox_max=data["bbox_max"],
                           notes=list(data.get("notes", [])))
        res.meshes = {k: MeshFile(**v) for k, v in data["meshes"].items()}
        res.instances = [Instance(**i) for i in data["instances"]]
        res._raw = data                                   # fuer den Cache-Vergleich
        return res
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _cached(out_dir: Path, src: Path, options: ImportOptions) -> ImportResult | None:
    res = load_manifest(out_dir)
    if res is None:
        return None
    raw = res._raw
    stamp = _source_stamp(src)
    if (raw.get("version") != IMPORT_VERSION or raw.get("options") != options.cache_key()
            or raw["source"].get("size") != stamp["size"]
            or raw["source"].get("mtime") != stamp["mtime"]):
        return None
    if not all((out_dir / f).is_file() for f in res.meshes):
        return None
    res.from_cache = True
    return res


_DEFAULTS_CACHE: dict = {}


def mesh_defaults(mesh_path) -> dict | None:
    """Farbe/Kollision eines CAD-Import-Meshes (fuer Editor und Generator).

    Der Scene Editor kennt fuer Meshes keine Farbe und exportiert jedes Mesh
    mit Kollision - ueber das Manifest im Mesh-Ordner werden die Werte aus dem
    CAD wiederhergestellt. None, wenn das Mesh nicht aus einem CAD-Import stammt.
    """
    p = Path(mesh_path)
    manifest = p.parent / MANIFEST_NAME
    try:
        mtime = manifest.stat().st_mtime
    except OSError:
        return None
    cached = _DEFAULTS_CACHE.get(manifest)
    if cached is None or cached[0] != mtime:
        try:
            meshes = json.loads(manifest.read_text(encoding="utf-8")).get("meshes", {})
        except (OSError, ValueError):
            return None
        cached = (mtime, meshes)
        _DEFAULTS_CACHE[manifest] = cached
    return cached[1].get(p.name)


def apply_mesh_defaults(root, xml_dir: Path, meshdir: Path | None = None) -> int:
    """Farbe/Kollision aus CAD-Manifesten in ein MuJoCo-XML eintragen.

    Nur Geoms, die selbst keine Angabe haben, werden ergaenzt (was im Editor
    oder von Hand gesetzt wurde, bleibt). -> Anzahl geaenderter Geoms.
    """
    base = Path(meshdir) if meshdir is not None else Path(xml_dir)
    files = {}
    changed = 0
    for asset in root.iter("asset"):
        for mesh in asset.findall("mesh"):
            if mesh.get("name") and mesh.get("file"):
                f = Path(os.path.expanduser(mesh.get("file")))
                f = f if f.is_absolute() else base / f
                files[mesh.get("name")] = f
                d = mesh_defaults(f)
                if d and d.get("shell") and mesh.get("inertia") is None:
                    mesh.set("inertia", "shell")     # sonst "mesh volume is too small"
                    changed += 1
    for geom in root.iter("geom"):
        f = files.get(geom.get("mesh") or "")
        if f is None:
            continue
        d = mesh_defaults(f)
        if not d:
            continue
        touched = False
        if geom.get("rgba") is None and geom.get("material") is None:
            geom.set("rgba", " ".join(f"{v:g}" for v in d["rgba"]))
            touched = True
        if not d.get("collision", True) and geom.get("contype") is None \
                and geom.get("conaffinity") is None:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            touched = True
        if not d.get("visual", True) and geom.get("group") is None:
            geom.set("group", COLLISION_GROUP)
            touched = True
        changed += touched
    return changed


# ---------------------------------------------------------------------------
# Umgebung schreiben
# ---------------------------------------------------------------------------
def _fmt(vals) -> str:
    return " ".join(f"{float(v):.6g}" for v in vals)


def environment_xml(result: ImportResult, xml_path: Path, place: str = DEFAULT_PLACEMENT) -> str:
    """Umgebungs-XML (nur Objekte, siehe README 'Basis + Umgebung')."""
    offset = result.placement_offset(place)
    root = ET.Element("mujoco", {"model": result.name})
    asset = ET.SubElement(root, "asset")
    mesh_names = {}
    used = set()
    for file in sorted(result.meshes):
        mname = _unique(f"{result.name}__{Path(file).stem}", used)
        mesh_names[file] = mname
        rel = os.path.relpath(result.out_dir / file, xml_path.parent).replace(os.sep, "/")
        attrs = {"name": mname, "file": rel}
        if result.meshes[file].shell:
            attrs["inertia"] = "shell"
        ET.SubElement(asset, "mesh", attrs)

    wb = ET.SubElement(root, "worldbody")
    body = ET.SubElement(wb, "body", {"name": result.name, "pos": _fmt(offset)})
    for inst in result.instances:
        attrs = {"name": inst.name, "type": "mesh", "mesh": mesh_names[inst.mesh],
                 "pos": _fmt(inst.pos)}
        if any(abs(a - b) > 1e-9 for a, b in zip(inst.quat, (1, 0, 0, 0))):
            attrs["quat"] = _fmt(inst.quat)
        attrs["rgba"] = _fmt(inst.rgba)
        if not inst.collision:
            attrs["contype"] = "0"
            attrs["conaffinity"] = "0"
        if not inst.visual:
            attrs["group"] = COLLISION_GROUP
        ET.SubElement(body, "geom", attrs)
    ET.indent(root, space="  ")

    header = (
        f"<!-- {_GENERATED_MARK} aus {Path(result.source).name}\n"
        f"     {result.summary()}\n"
        "     Nur Optik: contype/conaffinity=0 (kleine Teile). Konkave Teile sind\n"
        "     zusaetzlich in konvexe Kollisions-Stuecke '<teil>__k<n>' zerlegt\n"
        f"     (group {COLLISION_GROUP}, im Viewer ausgeblendet). Meshes in "
        f"{os.path.relpath(result.out_dir, xml_path.parent).replace(os.sep, '/')}/\n"
        f"     Platzierung '{place}': ganze Baugruppe per <body pos> verschiebbar.\n"
        "     Neu erzeugen:  ./launch.sh import <datei>  (ueberschreibt diese Datei) -->\n")
    return header + ET.tostring(root, encoding="unicode") + "\n"


_GENERATED_MARK = "AUTO-GENERIERT von scene_editor/cad_import.py"


def is_generated(xml_path: Path) -> bool:
    """Stammt die Umgebungs-XML unveraendert aus diesem Import?

    Speichert man eine importierte Umgebung im Editor (Objekte dazu, Teile
    verschoben), faellt der Kopfkommentar weg - dann darf ein erneuter Import
    sie nicht still ueberschreiben.
    """
    try:
        with open(xml_path, encoding="utf-8", errors="replace") as fh:
            return _GENERATED_MARK in fh.read(4096)
    except OSError:
        return False


def placement_notes(result: ImportResult, place: str) -> list:
    """Wo steht der G1 nach der Platzierung? Warnung, wenn in einem Teil."""
    offset = result.placement_offset(place)
    hits = result.spawn_conflicts(offset)
    spot = f"CAD-Position x={-offset[0]:.2f} y={-offset[1]:.2f} m"
    if not hits:
        return [f"Der G1 startet frei bei {spot} (Platzierung '{place}')."]
    more = f" und {len(hits) - 3} weitere" if len(hits) > 3 else ""
    return [f"ACHTUNG: Der G1 startet bei {spot} IN {', '.join(hits[:3])}{more}. "
            "Umgebung verschieben (im Editor die Gruppe, sonst <body pos> in der "
            "XML) oder mit Platzierung 'auto' importieren."]


def write_environment(result: ImportResult, xml_path: Path | None = None,
                      place: str = DEFAULT_PLACEMENT) -> Path:
    xml_path = Path(xml_path) if xml_path else env_store.env_xml(result.name)
    xml_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = xml_path.with_name(xml_path.name + ".tmp")
    tmp.write_text(environment_xml(result, xml_path, place), encoding="utf-8")
    os.replace(tmp, xml_path)
    return xml_path


# ---------------------------------------------------------------------------
# Einstieg
# ---------------------------------------------------------------------------
def import_file(src, name: str | None = None, options: ImportOptions | None = None,
                out_root: Path = CAD_DIR, log=_log) -> ImportResult:
    """CAD-/Mesh-Datei importieren -> Einzelteil-Meshes unter out_root/<name>/.

    Wirft ImportFailed mit Klartext, wenn es nicht geht. Ein gleicher Import
    (Datei unveraendert, gleiche Optionen) kommt aus dem Cache.
    """
    options = options or ImportOptions()
    options.validate()
    src = Path(src).expanduser().resolve()
    if not src.is_file():
        raise ImportFailed(f"Datei nicht gefunden: {src}")
    suffix = src.suffix.lower()
    if suffix not in CAD_SUFFIXES and suffix not in MESH_SUFFIXES:
        raise ImportFailed(
            f"{src.name}: Format {suffix or '(ohne Endung)'} wird nicht unterstuetzt. "
            f"Moeglich: {', '.join(sorted(set(CAD_SUFFIXES) | set(MESH_SUFFIXES)))}. "
            "JT/Parasolid/SolidWorks-Dateien bitte im CAD als STEP exportieren.")

    name = sanitize_name(name or src.stem, "cad_import")
    out_dir = Path(out_root) / name
    cached = _cached(out_dir, src, options)
    if cached is not None:
        log(f"{src.name}: unveraendert - vorhandener Import wird verwendet "
            f"({out_dir.name}/).")
        return cached

    t0 = time.time()
    if suffix in CAD_SUFFIXES:
        protos, raw, unit, notes = _read_occ(src, CAD_SUFFIXES[suffix], options, log)
    else:
        protos, raw, unit, notes = _read_trimesh(src, options, log)

    # Erst in einen Nachbarordner schreiben und dann tauschen: bricht der
    # Import ab, bleibt ein vorhandener alter Import intakt.
    tmp_dir = out_dir.with_name(out_dir.name + ".tmp")
    shutil.rmtree(tmp_dir, ignore_errors=True)
    tmp_dir.mkdir(parents=True)
    result = ImportResult(name=name, source=str(src), out_dir=tmp_dir, unit=unit,
                          notes=list(notes))
    try:
        _build(protos, raw, unit, options, tmp_dir, result, log)
        _write_manifest(result, options, src)
    except BaseException:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        raise
    if out_dir.exists():
        shutil.rmtree(out_dir)
    os.replace(tmp_dir, out_dir)
    result.out_dir = out_dir

    diag = math.dist(result.bbox_min, result.bbox_max)
    if suffix in MESH_SUFFIXES and options.scale is None and diag > 30.0:
        result.notes.append(f"Das Modell ist {diag:.0f} m gross - liegt es evtl. in mm "
                            "vor? Dann mit Skalierung 0.001 importieren.")
    result.seconds = time.time() - t0
    log(f"fertig in {result.seconds:.0f} s: {result.summary()}")
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="CAD/Mesh -> MuJoCo-Umgebung scenes/<name>/ (umgebung.xml + meshes/)")
    ap.add_argument("file", nargs="?", help="STEP/IGES/BREP oder STL/OBJ/PLY/GLB/...")
    ap.add_argument("--name", help="Name der Umgebung (Default: Dateiname)")
    ap.add_argument("--quality", choices=list(QUALITY), default=DEFAULT_QUALITY,
                    help="Feinheit der Tesselierung (Default: %(default)s)")
    ap.add_argument("--scale", type=float, default=None,
                    help="Skalierung erzwingen (Default: automatisch - CAD in "
                         "Metern, Meshes unveraendert)")
    ap.add_argument("--place", choices=PLACEMENTS, default=DEFAULT_PLACEMENT,
                    help="auto = Boden auf z=0 und G1 (Ursprung) auf freien Platz "
                         "(Default), floor = nur Boden auf z=0, center = zusaetzlich "
                         "mittig um den Ursprung, cad = Koordinaten unveraendert")
    ap.add_argument("--min-collision-size", type=float, default=DEFAULT_MIN_COLLISION_SIZE,
                    help="Teile mit kleinerer Diagonale (m) nur anzeigen "
                         "(Default: %(default)s)")
    ap.add_argument("--no-collision", action="store_true",
                    help="alles nur anzeigen (reine Deko-Umgebung)")
    ap.add_argument("--no-scene", action="store_true",
                    help="nur Meshes in die Zwischenablage meshes/cad/<name>/ "
                         "(zum Einfuegen im Editor), keine Umgebung")
    ap.add_argument("--force", action="store_true", help="Cache ignorieren, neu vernetzen")
    ap.add_argument("--replace", action="store_true",
                    help="eine im Editor weiterbearbeitete Umgebung gleichen Namens ersetzen")
    ap.add_argument("--check", action="store_true",
                    help="nur pruefen, ob das CAD-Backend installiert ist (inkl. coacd/trimesh; Exit 0/2)")
    args = ap.parse_args(argv)

    if args.check:
        # Alle drei, sonst holt launch.sh in einem aelteren venv (das von der
        # frueheren STEP-Konvertierung schon cadquery-ocp hat) coacd nie nach.
        return 0 if (occ_available() and trimesh_available() and coacd_available()) else 2
    if not args.file:
        ap.error("Datei fehlt")

    options = ImportOptions(quality=args.quality, scale=args.scale, place=args.place,
                            min_collision_size=args.min_collision_size,
                            collision=not args.no_collision)
    # Umgebungs-Name = Ordnername = G1_ENV -> dieselben strengen Regeln.
    name = env_store.sanitize_env_name(args.name or Path(args.file).stem) or "cad_import"
    folder = env_store.env_dir(name)
    xml = folder / env_store.ENV_FILE
    legacy = env_store.SCENES_DIR / f"{name}.xml"
    if not args.no_scene and not args.replace:
        if (xml.is_file() and not is_generated(xml)) or legacy.is_file():
            print(f"Fehler: Die Umgebung '{name}' gibt es schon und sie wurde nicht "
                  "(nur) von diesem Import erzeugt - vermutlich im Editor weiter"
                  "bearbeitet. Anderen Namen waehlen (--name) oder ersetzen "
                  "(--replace).", file=sys.stderr)
            return 1
    out_root = CAD_DIR if args.no_scene else folder / env_store.ENV_MESH_DIR
    if args.force:
        (out_root / name / MANIFEST_NAME).unlink(missing_ok=True)
    try:
        result = import_file(args.file, name, options, out_root=out_root)
    except ImportFailed as exc:
        print(f"Fehler: {exc}", file=sys.stderr)
        return 1
    print(f"OK: {result.summary()}")
    print(f"    Meshes: {result.out_dir}")
    if not args.no_scene:
        write_environment(result, xml, place=options.place)
        if legacy.is_file():
            legacy.unlink()               # altes Format gleichen Namens abloesen
        # Reste einer frueheren (Editor-)Fassung dieser Umgebung wegraeumen:
        # der Ordner soll genau das enthalten, was umgebung.xml benutzt.
        for extra in sorted(folder.rglob("*"), reverse=True):
            keep = (extra == xml or extra == result.out_dir
                    or result.out_dir in extra.parents or extra in result.out_dir.parents)
            if keep:
                continue
            if extra.is_file():
                extra.unlink()
            elif extra.is_dir() and not any(extra.iterdir()):
                extra.rmdir()
        print(f"    Umgebung: {xml}  (beim G1-Start als '{name}' waehlbar)")
    for note in result.notes + placement_notes(result, options.place):
        print(f"Hinweis: {note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
