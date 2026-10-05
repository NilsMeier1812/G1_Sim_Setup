#!/usr/bin/env python3
"""
Mesh-Pruefung fuer MuJoCo: STL lesen/schreiben/reparieren - nur Standardbibliothek.

Dieses Modul wird von allen Teilen des Scene Editors benutzt, auch vom blanken
System-python3 aus (start.sh -> build_env_scene.py), wo im Zweifel weder numpy
noch trimesh installiert sind. Darum bewusst ohne Abhaengigkeiten.

MuJoCo-Grenzen, um die es hier geht:
  * Mesh-Formate: nur STL, OBJ und MSH.
  * STL nur BINAER. ASCII-STL bricht mit "stl_decoder: number of faces should
    be between 1 and 200000 ... perhaps this is an ASCII file?" ab (MuJoCo liest
    den ASCII-Text als Face-Zaehler).
  * STL hoechstens 200000 Dreiecke - dieselbe Fehlermeldung.

Grosse Netze und alle anderen Formate (STEP, IGES, PLY, GLB, ...) wandelt
cad_import.py um; hier wird nur geprueft und (ASCII -> binaer) repariert.

    python3 mesh_utils.py [ordner|datei.stl]   # STLs pruefen, ASCII reparieren
                                             # (ohne Argument: meshes/ + scenes/)
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MESHES_DIR = HERE / "meshes"

#: MuJoCos harte Obergrenze fuer STL-Dreiecke.
MJ_MAX_FACES = 200_000

#: Mesh-Formate, die MuJoCo direkt laden kann.
MJ_MESH_SUFFIXES = (".stl", ".obj", ".msh")


def _de_number(n: int) -> str:
    """Zahl mit deutschen Tausenderpunkten (200.000)."""
    return f"{n:,}".replace(",", ".")


# ---------------------------------------------------------------------------
# Binaere STL
# ---------------------------------------------------------------------------
def stl_face_count(path) -> int | None:
    """Face-Anzahl einer BINAEREN STL - None, wenn die Datei nicht binaer ist.

    Binaeres STL: 80-Byte-Header + 4-Byte-Face-Anzahl + Faces*50 Byte, exakt
    passend zur Dateigroesse. ASCII-STL erfuellt das praktisch nie.
    """
    try:
        with open(path, "rb") as fh:
            head = fh.read(84)
        size = Path(path).stat().st_size
    except OSError:
        return None
    if len(head) < 84:
        return None
    ntri = struct.unpack_from("<I", head, 80)[0]
    if size != 84 + ntri * 50:
        return None
    return ntri


def is_binary_stl(path) -> bool:
    """True, wenn die Datei als binaere STL gelesen werden kann."""
    return stl_face_count(path) is not None


def is_valid_binary_stl(path) -> bool:
    """True, wenn MuJoCo diese Datei als STL laden kann (binaer, 1..200000)."""
    faces = stl_face_count(path)
    return faces is not None and 1 <= faces <= MJ_MAX_FACES


def write_binary_stl(dest, triangles) -> int:
    """Dreiecke als binaere STL schreiben. -> Anzahl Dreiecke.

    triangles: Iterable von ((x,y,z), (x,y,z), (x,y,z)). Die Normale wird aus
    der Eckreihenfolge berechnet (MuJoCo ignoriert sie ohnehin).
    """
    body = bytearray()
    n = 0
    for a, b, c in triangles:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        length = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
        body += struct.pack("<12fH", nx / length, ny / length, nz / length,
                            *a, *b, *c, 0)
        n += 1
    Path(dest).write_bytes(b"\0" * 80 + struct.pack("<I", n) + bytes(body))
    return n


def read_binary_stl_bounds(path):
    """(min_xyz, max_xyz) einer binaeren STL, None wenn nicht lesbar.

    Rein mit struct: fuer Aufrufer ohne numpy (build_env_scene/scene_objects).
    """
    faces = stl_face_count(path)
    if not faces:
        return None
    data = Path(path).read_bytes()
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for vals in struct.iter_unpack("<12fH", data[84:84 + faces * 50]):
        for k in (3, 6, 9):
            for i in range(3):
                v = vals[k + i]
                if v < lo[i]:
                    lo[i] = v
                if v > hi[i]:
                    hi[i] = v
    return tuple(lo), tuple(hi)


# ---------------------------------------------------------------------------
# ASCII-STL -> binaer
# ---------------------------------------------------------------------------
_NUM = r"[-+0-9.eEdD]+"
_FACET_RE = re.compile(
    r"facet\s+normal\s+(%s)\s+(%s)\s+(%s).*?outer\s+loop(.*?)endloop" % (_NUM, _NUM, _NUM),
    re.IGNORECASE | re.DOTALL)
_VERTEX_RE = re.compile(r"vertex\s+(%s)\s+(%s)\s+(%s)" % (_NUM, _NUM, _NUM), re.IGNORECASE)


def _f(s: str) -> float:
    # Fortran-Exponenten (1.0D-3) kommen in aelteren CAD-Exporten vor.
    return float(s.replace("D", "E").replace("d", "e"))


def ascii_stl_to_binary(src, dest=None) -> int:
    """ASCII-STL nach binaerem STL wandeln. Gibt die Face-Anzahl zurueck.

    Polygone mit mehr als drei Ecken (kommen in manchen Exporten vor) werden
    als Faecher trianguliert statt verworfen.
    """
    src = Path(src)
    dest = Path(dest) if dest is not None else src
    text = src.read_text(encoding="utf-8", errors="replace")

    tris = []
    for m in _FACET_RE.finditer(text):
        verts = [tuple(_f(c) for c in v) for v in _VERTEX_RE.findall(m.group(4))]
        for i in range(1, len(verts) - 1):
            tris.append((verts[0], verts[i], verts[i + 1]))

    if not tris:
        raise ValueError(f"{src.name}: keine Dreiecke gefunden (keine gueltige ASCII-STL?)")
    return write_binary_stl(dest, tris)


# ---------------------------------------------------------------------------
# "Kann MuJoCo das laden?" im Klartext
# ---------------------------------------------------------------------------
def stl_problem(path) -> str | None:
    """Warum MuJoCo diese STL NICHT laden kann (sonst None) - im Klartext."""
    name = Path(path).name
    faces = stl_face_count(path)
    if faces is None:
        return (f"'{name}' ist keine binaere STL (MuJoCo kann nur binaeres "
                "STL - die Meldung 'perhaps this is an ASCII file?' kommt genau "
                "daher).")
    if faces > MJ_MAX_FACES:
        return (f"'{name}' hat {_de_number(faces)} Dreiecke - MuJoCos Grenze "
                f"fuer STL liegt bei {_de_number(MJ_MAX_FACES)}.")
    if faces < 1:
        return f"'{name}' enthaelt keine Dreiecke."
    return None


def make_mujoco_ready(path, notes=None) -> str | None:
    """Mesh fuer MuJoCo brauchbar machen: ASCII-STL -> binaer, sonst Problem melden.

    Rueckgabe: None, wenn die Datei (danach) ladbar ist, sonst der Grund als
    Text. Hinweise ueber durchgefuehrte Reparaturen landen in `notes`.
    Zu grosse Netze und fremde Formate repariert cad_import.py (braucht numpy).
    """
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix != ".stl":
        if suffix not in MJ_MESH_SUFFIXES:
            return (f"'{path.name}': MuJoCo kann dieses Format nicht direkt laden "
                    f"(nur {', '.join(MJ_MESH_SUFFIXES)}) - ueber den CAD-Import "
                    "(cad_import.py) wird es automatisch umgewandelt.")
        return None                      # OBJ/MSH prueft MuJoCo selbst
    if stl_face_count(path) is None:
        try:
            faces = ascii_stl_to_binary(path)
        except (OSError, ValueError, struct.error) as exc:
            return f"'{path.name}' laesst sich nicht als STL lesen ({exc})."
        if notes is not None:
            notes.append(f"{path.name} war eine ASCII-STL und wurde binaer neu "
                         f"geschrieben ({_de_number(faces)} Dreiecke).")
    return stl_problem(path)


def check_meshes(target: Path) -> int:
    """Alle STL unter `target` pruefen (ASCII gleich reparieren). Exit-Code."""
    files = sorted(target.rglob("*.stl")) if target.is_dir() else [target]
    if not files:
        print(f"Keine STL-Dateien in {target}")
        return 0
    bad = 0
    for f in files:
        notes = []
        problem = make_mujoco_ready(f, notes)
        for note in notes:
            print(f"repariert: {note}")
        if problem:
            bad += 1
            print(f"PROBLEM  : {problem}", file=sys.stderr)
        else:
            print(f"ok       : {f.relative_to(target) if target.is_dir() else f.name} "
                  f"({_de_number(stl_face_count(f))} Dreiecke)")
    if bad:
        print(f"\n{bad} Datei(en) nicht MuJoCo-tauglich. Zu grosse Netze ueber den "
              "CAD-Import neu einlesen:  ./launch.sh import <datei>", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    # Ohne Argument: Mesh-Bibliothek UND alle Umgebungen (scenes/<name>/meshes/).
    _targets = [Path(a) for a in sys.argv[1:]] or [MESHES_DIR, HERE / "scenes"]
    raise SystemExit(max(check_meshes(t) for t in _targets))
