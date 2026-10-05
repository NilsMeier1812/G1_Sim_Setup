#!/usr/bin/env python3
"""
Kombiniert die feste G1-BASIS mit einer UMGEBUNG zu einer lauffaehigen Szene.

Idee des Systems:
  * BASIS (kommt IMMER automatisch dazu, hier fest verdrahtet):
      - der G1 (per <include>, Hand-Variante je nach --inspire)
      - Lichtquelle
      - Boden + Skybox/Groundplane
      - der Weld "hold_base_weld" (haelt den G1 am Anfang an torso_link fest;
        wird vom Sim per Name gesteuert, siehe hold_base.py)
      - visual/statistic-Grundeinstellungen
  * UMGEBUNG (scene_editor/scenes/<name>/umgebung.xml, Meshes daneben in
    meshes/ - siehe env_store.py): enthaelt NUR Hindernisse bzw. Objekte zum
    Interagieren/Greifen - also nur <asset> (eigene Meshes) und die Objekte
    im <worldbody>. KEIN Roboter, KEIN Boden, KEIN Licht, KEIN Weld.

Dieses Skript erzeugt die kombinierte Szene  unitree_robots/g1/scene_env_<name>.xml
= BASIS + Objekte der Umgebung.

NORMALISIERUNG (wichtig): die Umgebung darf aussehen, wie der Scene-Editor sie
exportiert - der Editor haengt naemlich JEDES per Maus angelegte Objekt in einen
eigenen <body> mit freiem Gelenk (<joint type="free"/>) und laesst die Pose am
GEOM stehen. Ungefiltert wuerde damit die halbe Umgebung beim Start umfallen.
Deshalb wird hier alles auf eine kanonische Form gebracht:

    Hindernis      ->  <geom name="X" pos=".." quat=".."/>   direkt im worldbody
    Greif-Objekt   ->  <body name="X" pos=".." quat="..">
                         <freejoint/><geom name="X" .../></body>

Klassifikation per Namenskonvention: ein Objekt-Name, der mit "grasp_" beginnt,
wird zum FREIEN Koerper (nur so kann MuJoCo es beim Greifen bewegen), alles
andere zum statischen Hindernis - unabhaengig davon, was der Editor exportiert
hat. Genau diese Form erwarten auch die Abnehmer:
unitree_mujoco/simulate_python/scene_objects.py (Sim-Seite, Live-Pose ueber den
BODY-Namen) und g1pilot/g1pilot/navigation/scene_bridge.py (ROS-Seite).

Ausserdem robust gemacht:
  * Mesh-Dateien, die ausserhalb von unitree_mujoco/ liegen (Editor schreibt
    absolute Pfade, z.B. Objaverse-Downloads), werden nach
    scene_editor/meshes/imported/ kopiert - sonst sind sie im read-only
    gemounteten Docker-Container unsichtbar.
  * Fehlende Meshes fuehren nicht mehr zu einer kaputten Szene: Asset UND die
    Geoms, die es benutzen, fallen raus (mit Warnung).
  * Namenskollisionen (mit der Basis oder untereinander) werden aufgeloest.
  * Das Ergebnis wird - falls mujoco importierbar ist - vor dem Schreiben
    kompiliert. Nur eine ladbare Szene wird geschrieben (atomar).

Ausserdem fuer Sim + Greifen:
  * STARTPUNKT: ein Objekt "g1_spawn..." legt fest, wo der G1 startet (x, y,
    Blickrichtung). Es wird entfernt und als <custom><numeric name="g1_spawn"
    data="x y yaw"/> abgelegt; unitree_mujoco.py setzt den Roboter dorthin.
  * KOLLISIONS-BITS: Umgebungs-Geoms bekommen contype/conaffinity 3 (Koerper
    UND Haende des G1), sonst greifen die Haende durch Box und Tisch.
  * Statische MESH-Hindernisse (nicht aus einem CAD-Import, der zerlegt selbst)
    kollidieren ueber eine konvexe Zerlegung (V-HACD) statt ueber EINE Huelle,
    die z.B. den Raum unter einer Tischplatte "auffuellt".

Aufruf:
    python3 build_env_scene.py --env warehouse                 (Name, sucht in scenes/)
    python3 build_env_scene.py --env warehouse --inspire 1
    python3 build_env_scene.py --env scenes/warehouse/umgebung.xml
"""
import argparse
import hashlib
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

GRASP_PREFIX_RE = re.compile(r"^grasp_", re.IGNORECASE)
# Startpunkt-Markierung: ein Objekt, dessen Name mit "g1_spawn" beginnt, legt
# fest, WO der G1 startet (Position x/y + Blickrichtung aus der Drehung um z).
# Es wird beim Kombinieren entfernt und als <custom><numeric name="g1_spawn"
# data="x y yaw"/> abgelegt; unitree_mujoco.py setzt den Roboter dorthin.
SPAWN_PREFIX_RE = re.compile(r"^g1_spawn", re.IGNORECASE)

HERE = Path(__file__).resolve().parent          # .../unitree_mujoco/scene_editor

# STL-Pruefung/ASCII-Reparatur (mesh_utils) und die Metadaten aus CAD-Importen
# (cad_import) haengen beim Import an nichts ausser der Standardbibliothek -
# laufen also auch unter dem System-python3, mit dem start.sh dieses Skript
# aufruft.
sys.path.insert(0, str(HERE))
import cad_import  # noqa: E402
import env_store  # noqa: E402
import mesh_utils  # noqa: E402

MJ_ROOT = HERE.parent                            # .../unitree_mujoco
G1_DIR = MJ_ROOT / "unitree_robots" / "g1"
G1_MESHDIR = G1_DIR / "meshes"                    # meshdir des Robotermodells
SCENES_DIR = HERE / "scenes"
MESHES_DIR = HERE / "meshes"
# Meshes, die von ausserhalb des Repos referenziert werden, landen hier (damit
# der Docker-Container sie ueber den /unitree_mujoco-Mount sieht).
IMPORTED_MESHES_DIR = MESHES_DIR / "imported"

# Kollisions-Zerlegung statischer Mesh-Hindernisse (siehe add_collision_hulls).
# Liegt unter unitree_mujoco/ -> auch im read-only gemounteten Container sichtbar.
COLLISION_CACHE = MESHES_DIR / ".collision"
# Namens-Suffix des erzeugten Kollisions-Bodys. scene_objects.py (Sim -> RViz)
# erkennt ihn daran, blendet ihn aus und behandelt das zugehoerige Optik-Mesh
# (contype/conaffinity 0) trotzdem als Hindernis. Beide Stellen muessen passen.
COLLISION_BODY_SUFFIX = "__collision"
# contype/conaffinity fuer Umgebungs-Objekte: Bit 1 (G1-Koerper) + Bit 2 (Haende).
ENV_COLLISION_BITS = "3"
CONTACT_ATTRS = ("friction", "solref", "solimp", "condim", "priority", "margin")

# Roboter-Basismodell je nach Hand-Variante (wie config.py)
ROBOT_FILES = {
    "0": "g1_29dof.xml",
    "1": "g1_29dof_inspire_ftp.xml",
}

# Namen, die die BASIS selbst vergibt - Umgebungs-Objekte duerfen sie nicht
# nochmal benutzen (MuJoCo bricht bei doppelten Namen hart ab).
BASE_GEOM_NAMES = {"floor"}
BASE_ASSET_NAMES = {"groundplane"}

# Orientierungs-Attribute, die wir beim Zusammenrechnen von Posen NICHT
# unterstuetzen (der Editor schreibt immer quat; Handarbeit kann anderes liefern).
UNSUPPORTED_ORIENT = ("euler", "axisangle", "xyaxes", "zaxis")

IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# kleine Helfer (Zahlen / Quaternionen) - bewusst ohne numpy, damit dieses
# Skript mit dem blanken System-python3 laeuft (start.sh ruft es so auf).
# ---------------------------------------------------------------------------
def _floats(s, default):
    if not s:
        return list(default)
    try:
        return [float(x) for x in s.split()]
    except ValueError:
        return list(default)


def _fmt(vals) -> str:
    return " ".join(f"{v:.6g}" for v in vals)


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _qrot(q, v):
    w, x, y, z = q
    qv = (0.0, v[0], v[1], v[2])
    qc = (w, -x, -y, -z)
    _, rx, ry, rz = _qmul(_qmul(q, qv), qc)
    return (rx, ry, rz)


def _compose(base_pos, base_quat, local_pos, local_quat):
    r = _qrot(base_quat, local_pos)
    pos = (base_pos[0] + r[0], base_pos[1] + r[1], base_pos[2] + r[2])
    return pos, _qmul(base_quat, local_quat)


def _is_identity(pos, quat) -> bool:
    return (all(abs(p) < 1e-12 for p in pos)
            and abs(quat[0] - 1.0) < 1e-12
            and all(abs(q) < 1e-12 for q in quat[1:]))


def _local_pose(el):
    pos = _floats(el.get("pos"), (0.0, 0.0, 0.0))[:3]
    quat = _floats(el.get("quat"), IDENTITY_QUAT)[:4]
    while len(pos) < 3:
        pos.append(0.0)
    if len(quat) < 4:
        quat = list(IDENTITY_QUAT)
    return tuple(pos), tuple(quat)


def _has_unsupported_orientation(el) -> bool:
    return any(el.get(a) for a in UNSUPPORTED_ORIENT)


def rel_to_meshdir(mesh_abs: Path) -> str:
    """Pfad relativ zu g1/meshes, plattform-neutral (mit '/')."""
    return os.path.relpath(mesh_abs, G1_MESHDIR).replace(os.sep, "/")


def sanitize_name(raw: str) -> str:
    """MuJoCo-taugliche Objekt-Namen. Der Editor legt Blueprint-Pfade wie
    '/box_0001' an - fuehrende Slashes und Leerzeichen muessen weg."""
    name = (raw or "").strip().strip("/")
    name = name.replace("/", "_")
    name = re.sub(r"[^A-Za-z0-9_.\-]", "_", name)
    name = re.sub(r"_{2,}", "_", name).strip("_")
    return name


def _unique(name: str, used: set) -> str:
    if name not in used:
        used.add(name)
        return name
    i = 2
    while f"{name}_{i}" in used:
        i += 1
    out = f"{name}_{i}"
    used.add(out)
    return out


def is_grasp_name(name: str) -> bool:
    return bool(GRASP_PREFIX_RE.match(name or ""))


def is_spawn_name(name: str) -> bool:
    return bool(SPAWN_PREFIX_RE.match(name or ""))


def _yaw(quat) -> float:
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------------------
# BASIS
# ---------------------------------------------------------------------------
def build_base(robot_file: str, model_name: str):
    """Baut die feste Basis (G1 + Licht + Boden + Weld + Technik).

    Gibt (mujoco_root, asset_element, worldbody_element) zurueck, damit die
    Objekte der Umgebung anschliessend in asset/worldbody eingemischt werden.
    """
    mj = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(mj, "include", {"file": robot_file})
    ET.SubElement(mj, "statistic", {"center": "0 0 0.5", "extent": "2.0"})

    vis = ET.SubElement(mj, "visual")
    ET.SubElement(vis, "headlight",
                  {"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(vis, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(vis, "global", {"azimuth": "-130", "elevation": "-20"})

    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "texture",
                  {"type": "skybox", "builtin": "gradient",
                   "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture",
                  {"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge",
                   "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                   "width": "300", "height": "300"})
    ET.SubElement(asset, "material",
                  {"name": "groundplane", "texture": "groundplane", "texuniform": "true",
                   "texrepeat": "5 5", "reflectance": "0.2"})

    wb = ET.SubElement(mj, "worldbody")
    # Lichtquelle (Basis)
    ET.SubElement(wb, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})
    # Boden (Basis)
    ET.SubElement(wb, "geom",
                  {"name": "floor", "size": "0 0 0.05", "type": "plane", "material": "groundplane"})

    # Weld, der den G1 am Anfang festhaelt (per Name vom Sim gesteuert).
    eq = ET.SubElement(mj, "equality")
    ET.SubElement(eq, "weld",
                  {"name": "hold_base_weld", "body1": "torso_link",
                   "solref": "0.01 1", "solimp": "0.99 0.999 0.001 0.5 2"})

    return mj, asset, wb


# ---------------------------------------------------------------------------
# UMGEBUNG einsammeln + normalisieren
# ---------------------------------------------------------------------------
def _joint_kinds(body_el):
    """Gelenke eines <body>: ('free', 'other') als Flags."""
    has_free = False
    has_other = False
    for child in body_el:
        if child.tag == "freejoint":
            has_free = True
        elif child.tag == "joint":
            if (child.get("type") or "hinge") == "free":
                has_free = True
            else:
                has_other = True
    return has_free, has_other


def collect_objects(container, base_pos, base_quat, warnings, out, depth=0,
                    body_name=None):
    """Laeuft den <worldbody>-Baum der Umgebung ab und sammelt flach ein.

    out bekommt Eintraege
        ("geom", geom_element, world_pos, world_quat, body_name_or_None)
        ("raw",  element)          - unveraenderte Uebernahme (z.B. Kamera,
                                     artikulierter Koerper)
    """
    for el in list(container):
        tag = el.tag
        if tag in ("light", "freejoint", "joint", "inertial", "site"):
            # Licht kommt aus der Basis; Gelenke/Inertials werden aus der
            # Klassifikation (grasp_) neu abgeleitet.
            continue

        if tag == "geom":
            if el.get("type") == "plane":
                continue  # Boden kommt aus der Basis
            if _has_unsupported_orientation(el) and not _is_identity(base_pos, base_quat):
                warnings.append(
                    f"  ! Geom '{el.get('name')}' benutzt {UNSUPPORTED_ORIENT} - Pose wird "
                    "nicht mit der Body-Pose verrechnet (bitte quat verwenden).")
                out.append(("raw", el))
                continue
            lp, lq = _local_pose(el)
            wpos, wquat = _compose(base_pos, base_quat, lp, lq)
            out.append(("geom", el, wpos, wquat, body_name))
            continue

        if tag == "body":
            _has_free, has_other = _joint_kinds(el)
            lp, lq = _local_pose(el)
            if has_other or _has_unsupported_orientation(el):
                # Artikuliertes Objekt (oder exotische Orientierung): unveraendert
                # uebernehmen, nur die aufsummierte Pose hineinschreiben.
                clone = el  # ET-Elemente werden ohnehin nur einmal eingehaengt
                if not _is_identity(base_pos, base_quat):
                    if _has_unsupported_orientation(el):
                        warnings.append(
                            f"  ! Body '{el.get('name')}' benutzt {UNSUPPORTED_ORIENT} in einer "
                            "verschachtelten Gruppe - Pose bleibt unveraendert.")
                    else:
                        wpos, wquat = _compose(base_pos, base_quat, lp, lq)
                        clone.set("pos", _fmt(wpos))
                        clone.set("quat", _fmt(wquat))
                if has_other:
                    warnings.append(
                        f"  ! Body '{el.get('name')}' hat eigene Gelenke -> wird unveraendert "
                        "uebernommen (keine Hindernis/Greif-Klassifikation).")
                out.append(("raw", clone))
                continue
            wpos, wquat = _compose(base_pos, base_quat, lp, lq)
            # Body-Name durchreichen: der Editor nennt den Wrapper
            # "<geom>_body_<n>" - fuer die Klassifikation zaehlt jeder von beiden.
            collect_objects(el, wpos, wquat, warnings, out, depth + 1,
                            body_name=el.get("name") or body_name)
            continue

        if depth == 0:
            # Kameras u.ae. auf oberster Ebene unveraendert uebernehmen.
            out.append(("raw", el))
        else:
            warnings.append(f"  ! <{tag}> in einer Gruppe wird ignoriert.")


def _strip_zero_mass(geom_el) -> None:
    """mass="0" an einem Geom, das gleich in einen FREIEN Koerper wandert, waere
    ein Koerper ohne Masse -> MuJoCo bricht beim Kompilieren ab. Attribut weg =
    MuJoCo rechnet Masse aus Geometrie + Dichte."""
    try:
        if float(geom_el.get("mass", "1")) <= 0.0:
            geom_el.attrib.pop("mass", None)
    except ValueError:
        geom_el.attrib.pop("mass", None)


def _ensure_geom_type(geom_el) -> None:
    """MuJoCos Default-Geomtyp ist 'sphere' - der Editor laesst type= bei Kugeln
    weg. Wir schreiben ihn immer explizit hin, damit die Abnehmer (RViz-Marker,
    Nav-Raster) den Typ nicht raten muessen."""
    if not geom_el.get("type"):
        geom_el.set("type", "sphere")


def _make_static_geom(geom_el, name, pos, quat):
    g = ET.Element("geom", dict(geom_el.attrib))
    g.set("name", name)
    _ensure_geom_type(g)
    g.set("pos", _fmt(pos))
    if _is_identity((0.0, 0.0, 0.0), quat):
        g.attrib.pop("quat", None)
    else:
        g.set("quat", _fmt(quat))
    return g


def _make_grasp_body(geom_el, name, pos, quat):
    """<body name="X" pos quat><freejoint/><geom name="X"/></body>.

    Body UND Geom tragen denselben Namen (in MuJoCo erlaubt - getrennte
    Namensraeume); die Sim-Seite (scene_objects.py/scene_state_publisher.py)
    loest die LIVE-Pose ueber den BODY-Namen auf.
    """
    attrs = {"name": name, "pos": _fmt(pos)}
    if not _is_identity((0.0, 0.0, 0.0), quat):
        attrs["quat"] = _fmt(quat)
    body = ET.Element("body", attrs)
    ET.SubElement(body, "freejoint")

    inner = ET.Element("geom", dict(geom_el.attrib))
    inner.attrib.pop("pos", None)
    inner.attrib.pop("quat", None)
    inner.set("name", name)
    _ensure_geom_type(inner)
    _strip_zero_mass(inner)
    body.append(inner)
    return body


# MuJoCos eigene Grenze fuer die Face-Anzahl in binaeren STL-Dateien (siehe
# dessen Fehlermeldung "number of faces should be between 1 and 200000").
_MJ_STL_MAX_FACES = mesh_utils.MJ_MAX_FACES
is_valid_binary_stl = mesh_utils.is_valid_binary_stl


def convert_stl_to_binary(path: Path, warnings) -> bool:
    """Versucht, eine ASCII-/kaputte STL binaer neu zu schreiben.

    Rueckgabe: True, wenn `path` danach eine gueltige binaere STL ist.
    Erst der eingebaute (abhaengigkeitsfreie) ASCII-Parser, dann - falls
    vorhanden - trimesh fuer exotischere Faelle. Zu grosse Netze kann keiner
    von beiden retten; dann gibt es eine Warnung mit klarer Ansage.
    """
    notes = []
    try:
        mesh_utils.ascii_stl_to_binary(path)
    except Exception as exc:
        first_error = exc
        try:
            import trimesh
            mesh = trimesh.load_mesh(str(path), file_type="stl")
            path.write_bytes(trimesh.exchange.stl.export_stl(mesh))
        except Exception:
            warnings.append(f"  ! '{path.name}' laesst sich nicht als STL "
                            f"lesen/reparieren ({first_error}) -> Objekt(e) damit "
                            "werden weggelassen.")
            return False
    else:
        notes.append("war eine ASCII-STL -> automatisch binaer neu geschrieben")

    faces = mesh_utils.stl_face_count(path)
    if faces is not None and faces > _MJ_STL_MAX_FACES:
        warnings.append(
            f"  ! '{path.name}' hat {faces} Dreiecke, MuJoCo kann hoechstens "
            f"{_MJ_STL_MAX_FACES} -> Objekt(e) damit werden weggelassen. "
            "Die Datei ueber den CAD-Import einlesen, der zerlegt sie: "
            "./launch.sh import <datei>")
        return False
    if not is_valid_binary_stl(path):
        warnings.append(f"  ! '{path.name}' bleibt nach der Reparatur ungueltig "
                        "-> Objekt(e) damit werden weggelassen.")
        return False
    warnings.append(f"  i '{path.name}' {', '.join(notes) or 'wurde repariert'} "
                    "(MuJoCo kann nur binaeres STL).")
    return True


def _resolve_mesh(file_attr: str, env_dir: Path, warnings):
    """Mesh-Datei finden, ggf. ins Repo kopieren und auf ladbares binaeres STL
    pruefen/reparieren.

    Rueckgabe: absoluter Pfad oder None (nicht verwendbar - der Grund steht
    dann bereits in `warnings`).
    """
    p = Path(os.path.expanduser(file_attr))
    candidates = [p] if p.is_absolute() else [env_dir / p, MESHES_DIR / p.name]
    mesh_abs = None
    for c in candidates:
        try:
            rc = c.resolve()
        except OSError:
            continue
        if rc.is_file():
            mesh_abs = rc
            break
    if mesh_abs is None:
        warnings.append(f"  ! Mesh-Datei nicht gefunden: {file_attr}")
        return None

    try:
        mesh_abs.relative_to(MJ_ROOT)
        final = mesh_abs
    except ValueError:
        # Ausserhalb von unitree_mujoco/ -> ins Repo kopieren, sonst sieht der
        # (read-only) Docker-Container die Datei nicht.
        try:
            IMPORTED_MESHES_DIR.mkdir(parents=True, exist_ok=True)
            dest = IMPORTED_MESHES_DIR / mesh_abs.name
            if not dest.is_file() or dest.stat().st_size != mesh_abs.stat().st_size:
                shutil.copy2(mesh_abs, dest)
            warnings.append(f"  i Mesh '{mesh_abs.name}' nach meshes/imported/ kopiert "
                            "(lag ausserhalb von unitree_mujoco/).")
            final = dest.resolve()
        except OSError as exc:
            warnings.append(f"  ! Mesh '{mesh_abs}' konnte nicht ins Repo kopiert werden: {exc}")
            return None

    if final.suffix.lower() == ".stl" and not is_valid_binary_stl(final):
        if not convert_stl_to_binary(final, warnings):
            return None

    return final


def merge_assets(env_root, asset, env_dir, warnings):
    """<asset> der Umgebung einmischen.

    Rueckgabe: (rename_map, dropped_meshes)
      rename_map      alter Asset-Name -> neuer Name (bei Kollision mit der Basis)
      dropped_meshes  Mesh-Namen, deren Datei fehlt (Geoms dazu fliegen raus)
    """
    used = {el.get("name") for el in asset if el.get("name")} | set(BASE_ASSET_NAMES)
    used.discard(None)
    rename_map = {}
    dropped = set()
    new_elements = []

    for env_asset in env_root.findall("asset"):
        for el in list(env_asset):
            if el.tag == "texture" and el.get("type") == "skybox":
                continue  # nur ein Skybox erlaubt (Basis hat schon einen)
            nm = el.get("name")

            if el.tag == "mesh":
                file_attr = el.get("file")
                if not file_attr:
                    warnings.append(f"  ! Mesh-Asset '{nm}' ohne file= -> uebersprungen.")
                    if nm:
                        dropped.add(nm)
                    continue
                mesh_abs = _resolve_mesh(file_attr, env_dir, warnings)
                if mesh_abs is None:
                    # _resolve_mesh hat den Grund schon in warnings eingetragen.
                    if nm:
                        dropped.add(nm)
                    continue
                el.set("file", rel_to_meshdir(mesh_abs))

            if nm:
                new_name = _unique(nm, used)
                if new_name != nm:
                    warnings.append(f"  i Asset-Name '{nm}' kollidiert -> '{new_name}'.")
                    rename_map[nm] = new_name
                    el.set("name", new_name)
            new_elements.append(el)

    # Referenzen INNERHALB der Assets nachziehen (Material -> Textur).
    for el in new_elements:
        for attr in ("texture", "material", "mesh"):
            ref = el.get(attr)
            if ref and ref in rename_map:
                el.set(attr, rename_map[ref])
        asset.append(el)

    return rename_map, dropped


def _decompose_cached(mesh_abs: Path):
    """Konvexe Zerlegung (V-HACD) eines Meshes, gecacht nach Datei-Inhalt.

    Gibt die Liste der Teil-Huellen (STL-Pfade) zurueck; None, wenn trimesh/
    vhacdx fehlen (dann bleibt das Mesh wie bisher).
    """
    digest = hashlib.sha1(mesh_abs.read_bytes()).hexdigest()[:10]
    out_dir = COLLISION_CACHE / f"{mesh_abs.stem}-{digest}"
    hulls = sorted(out_dir.glob("hull_*.stl"))
    if hulls:
        return hulls
    try:
        import trimesh
    except ImportError:
        return None
    try:
        mesh = trimesh.load(str(mesh_abs), force="mesh")
        parts = trimesh.decomposition.convex_decomposition(
            mesh, maxConvexHulls=64, resolution=400000, maxRecursionDepth=12)
    except Exception:  # vhacdx fehlt / Mesh nicht zerlegbar
        return None
    if isinstance(parts, dict):
        parts = [parts]
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, part in enumerate(parts):
        trimesh.Trimesh(part["vertices"], part["faces"]).export(
            str(out_dir / f"hull_{i:02d}.stl"))
    return sorted(out_dir.glob("hull_*.stl"))


def _needs_collision_hulls(geom_el, mesh_abs) -> bool:
    """Statisches Mesh-Hindernis, das (noch) ueber EINE konvexe Huelle kollidiert?

    Nicht zerlegt werden: reine Optik (contype/conaffinity 0), Kollisions-
    Stuecke (group 3) und Meshes aus einem CAD-Import - cad_import.py zerlegt
    konkave Teile schon selbst (CoACD, Stuecke '<teil>__k<n>').
    """
    if geom_el.get("type") != "mesh" or mesh_abs is None:
        return False
    if geom_el.get("contype") == "0" and geom_el.get("conaffinity") == "0":
        return False
    if geom_el.get("group") == cad_import.COLLISION_GROUP:
        return False
    return cad_import.mesh_defaults(mesh_abs) is None


def add_collision_hulls(geom_el, mesh_abs: Path, mesh_scale, asset, wb, warnings,
                        hull_assets: dict) -> None:
    """Statisches Mesh-Hindernis: Kollision ueber konvexe Teil-Huellen.

    MuJoCo kollidiert ein Mesh-Geom nur mit seiner EINEN konvexen Huelle. Bei
    einem Arbeitsplatz/Regal "fuellt" die alles auf: Objekte auf dem Tisch
    starten IN der Huelle und werden weggeschossen, auf der Tischplatte liegen
    bleibt nichts. Darum: das Original-Mesh nur noch als Optik (contype/
    conaffinity 0), die Kollision uebernehmen die Teile der Zerlegung (group 3
    = im Viewer standardmaessig ausgeblendet) in einem Body mit derselben Pose.

    hull_assets: Mesh-Name -> Asset-Namen der Teile (je Mesh nur EINMAL
    angelegt, auch wenn mehrere Geoms dasselbe Mesh benutzen).
    """
    mesh_name = geom_el.get("mesh")
    if mesh_name not in hull_assets:
        hulls = _decompose_cached(mesh_abs)
        if not hulls:
            warnings.append(f"  ! Mesh '{mesh_name}': keine Kollisions-Zerlegung "
                            "(trimesh/vhacdx fehlen) -> Kollision nur ueber konvexe Huelle.")
            hull_assets[mesh_name] = None
            return
        names = []
        for i, hull in enumerate(hulls):
            hull_name = f"{mesh_name}_hull{i:02d}"
            attrs = {"name": hull_name, "file": rel_to_meshdir(hull)}
            if mesh_scale:
                attrs["scale"] = mesh_scale
            ET.SubElement(asset, "mesh", attrs)
            names.append(hull_name)
        hull_assets[mesh_name] = names
        warnings.append(f"  i Mesh '{mesh_name}': Kollision ueber {len(names)} "
                        "konvexe Teile (Viewer: Gruppe 3 einblenden zum Ansehen).")
    names = hull_assets[mesh_name]
    if not names:
        return

    geom_el.set("contype", "0")
    geom_el.set("conaffinity", "0")

    body = ET.SubElement(wb, "body", {"name": f"{geom_el.get('name')}{COLLISION_BODY_SUFFIX}",
                                      "pos": geom_el.get("pos", "0 0 0")})
    if geom_el.get("quat"):
        body.set("quat", geom_el.get("quat"))
    for hull_name in names:
        hull_attrs = {"type": "mesh", "mesh": hull_name, "group": "3",
                      "rgba": "0.9 0.3 0.3 0.4"}
        for attr in CONTACT_ATTRS:
            if geom_el.get(attr):
                hull_attrs[attr] = geom_el.get(attr)
        ET.SubElement(body, "geom", hull_attrs)


def merge_environment(env_root, asset, wb, env_dir, warnings, spawn_out=None):
    """Mischt die Objekte der Umgebung normalisiert in die Basis ein.

    spawn_out: Liste, an die der Startpunkt (x, y, yaw) eines "g1_spawn"-
    Objekts angehaengt wird (das Objekt selbst kommt nicht in die Szene).
    """
    # Meshes aus CAD-Importen: Farbe, "nur Optik", Kollisions-Stuecke und
    # inertia="shell" (flache Teile) aus dem Import-Manifest nachtragen, falls
    # die Umgebung sie nicht selbst setzt (z.B. nach einem Editor-Durchlauf).
    cad_import.apply_mesh_defaults(env_root, env_dir)
    rename_map, dropped_meshes = merge_assets(env_root, asset, env_dir, warnings)

    n_base = len(wb)  # alles danach in <worldbody> stammt aus der Umgebung
    # Mesh-Name -> (absolute Datei, scale) fuer die Kollisions-Zerlegung
    mesh_files = {
        el.get("name"): ((G1_MESHDIR / el.get("file")).resolve(), el.get("scale"))
        for el in asset.findall("mesh") if el.get("name") and el.get("file")
    }
    hull_assets = {}

    collected = []
    for env_wb in env_root.findall("worldbody"):
        collect_objects(env_wb, (0.0, 0.0, 0.0), IDENTITY_QUAT, warnings, collected)

    # EIN gemeinsamer Namenstopf fuer Geoms und Bodies: Greif-Objekte tragen
    # denselben Namen an Body UND Geom, damit bleibt garantiert alles eindeutig.
    used_names = set(BASE_GEOM_NAMES)
    n_obstacles = 0
    n_grasp = 0
    auto_idx = 0

    for entry in collected:
        if entry[0] == "raw":
            wb.append(entry[1])
            continue

        _, geom_el, wpos, wquat, body_name = entry

        # Asset-Umbenennungen nachziehen.
        for attr in ("mesh", "material"):
            ref = geom_el.get(attr)
            if ref and ref in rename_map:
                geom_el.set(attr, rename_map[ref])

        mesh_ref = geom_el.get("mesh")
        if mesh_ref and mesh_ref in dropped_meshes:
            warnings.append(f"  ! Objekt '{geom_el.get('name')}' benutzt ein fehlendes Mesh "
                            "-> weggelassen.")
            continue

        raw_name = geom_el.get("name") or body_name or ""
        name = sanitize_name(raw_name)
        if not name:
            auto_idx += 1
            name = f"env_object_{auto_idx}"

        # Startpunkt-Markierung: nur Position + Blickrichtung zaehlen, das
        # Objekt selbst gehoert nicht in die Szene (kein Hindernis).
        if is_spawn_name(name) or is_spawn_name(sanitize_name(body_name or "")):
            if spawn_out is not None and not spawn_out:
                spawn = (wpos[0], wpos[1], _yaw(wquat))
                spawn_out.append(spawn)
                warnings.append(f"  i Startpunkt '{name}': G1 startet bei x={spawn[0]:.2f} "
                                f"y={spawn[1]:.2f}, Blickrichtung "
                                f"{math.degrees(spawn[2]):.0f} Grad.")
            elif spawn_out:
                warnings.append(f"  ! Weiterer Startpunkt '{name}' ignoriert "
                                "(es zaehlt der erste).")
            continue

        # grasp_ zaehlt, egal ob es am Geom oder am umschliessenden Body steht
        # (der Editor haengt an den Wrapper-Body ein "_body_<n>" an).
        grasp = is_grasp_name(name) or is_grasp_name(sanitize_name(body_name or ""))

        name = _unique(name, used_names)
        if grasp:
            wb.append(_make_grasp_body(geom_el, name, wpos, wquat))
            n_grasp += 1
        else:
            g = _make_static_geom(geom_el, name, wpos, wquat)
            wb.append(g)
            n_obstacles += 1
            mesh_abs, mesh_scale = mesh_files.get(g.get("mesh"), (None, None))
            if _needs_collision_hulls(g, mesh_abs):
                used_names.add(f"{name}{COLLISION_BODY_SUFFIX}")
                add_collision_hulls(g, mesh_abs, mesh_scale, asset, wb, warnings,
                                    hull_assets)

    # Kollisions-Bits der Umgebung: Koerper-Geoms des G1 kollidieren auf Bit 1,
    # die Haende/Finger (Inspire) nur auf Bit 2 (damit sie nicht am eigenen
    # Koerper haengen bleiben). Umgebungs-Objekte bekommen darum BEIDE Bits (3),
    # sonst greifen die Haende durch Box und Tisch hindurch. Explizit gesetzte
    # Werte (z.B. contype=0 = reine Optik) bleiben unangetastet.
    for el in list(wb)[n_base:]:
        for g in el.iter("geom"):
            if g.get("contype") is None and g.get("conaffinity") is None:
                g.set("contype", ENV_COLLISION_BITS)
                g.set("conaffinity", ENV_COLLISION_BITS)

    # Auf Abschnitte hinweisen, die eine reine Objekt-Umgebung normalerweise
    # nicht enthalten sollte (werden bewusst NICHT uebernommen). <keyframe>,
    # <size> und <compiler> schreibt der Editor immer mit - die passen zur
    # kombinierten Szene ohnehin nicht und werden still ignoriert.
    for tag in ("equality", "actuator", "default", "contact", "tendon", "sensor"):
        if env_root.find(tag) is not None:
            warnings.append(f"  ! <{tag}> in der Umgebung wird ignoriert "
                            "(Umgebungen sollen nur Objekte enthalten).")

    return n_obstacles, n_grasp


# ---------------------------------------------------------------------------
# Ein-/Ausgabe
# ---------------------------------------------------------------------------
def resolve_env_path(arg: str) -> Path:
    """Umgebung finden (Name, Ordner oder Pfad, siehe env_store.resolve).
    Nicht gefunden -> ein nicht existierender Pfad (fuer die Fehlermeldung)."""
    found = env_store.resolve(arg)
    return found if found is not None else Path(os.path.expanduser(arg)).resolve()


def available_envs() -> list:
    return env_store.names()


def parse_environment(env_path: Path):
    """Umgebungs-XML einlesen (Kommentare vorher strippen: MuJoCos Parser ist
    tolerant, ElementTree strikt - z.B. sind '--'-Folgen in XML-Kommentaren
    ungueltig)."""
    raw = env_path.read_text(encoding="utf-8", errors="replace")
    cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
    root = ET.fromstring(cleaned)
    if root.tag != "mujoco":
        raise ValueError(f"Wurzel-Element ist <{root.tag}>, erwartet <mujoco>")
    return root


def validate_scene(path: Path):
    """Kombinierte Szene testweise kompilieren. Rueckgabe: Fehlertext oder None.
    Ist mujoco nicht installiert (blankes System-python3), wird uebersprungen."""
    try:
        import mujoco  # noqa: F401
    except Exception:
        return None
    try:
        mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:  # pragma: no cover - haengt an der Szene
        return str(exc)
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description="G1-Basis + Umgebung zu lauffaehiger Szene kombinieren")
    ap.add_argument("--env", required=True,
                    help="Umgebung: Pfad, Dateiname oder blosser Name (z.B. scenes/warehouse.xml)")
    ap.add_argument("--inspire", default="0", choices=["0", "1"],
                    help="0 = Rubber-Hand-G1, 1 = Inspire-FTP-G1 (Default 0)")
    ap.add_argument("--out",
                    help="Zieldatei (Default: unitree_robots/g1/scene_env_<name>.xml)")
    ap.add_argument("--no-validate", action="store_true",
                    help="Ergebnis NICHT testweise mit MuJoCo kompilieren")
    args = ap.parse_args()

    env_path = resolve_env_path(args.env)
    if not env_path.is_file():
        envs = available_envs()
        hint = ("Vorhandene Umgebungen: " + ", ".join(envs)) if envs else \
               "Es gibt noch keine Umgebung in scene_editor/scenes/."
        sys.exit(f"[build_env_scene] Umgebung nicht gefunden: {env_path}\n{hint}")
    env_dir = env_path.parent

    robot_file = ROBOT_FILES[args.inspire]
    if not (G1_DIR / robot_file).is_file():
        sys.exit(f"[build_env_scene] Robotermodell fehlt: {G1_DIR / robot_file}")

    name = env_store.env_name_of(env_path)
    out_path = Path(args.out).resolve() if args.out else (G1_DIR / f"scene_env_{name}.xml")

    try:
        env_root = parse_environment(env_path)
    except (OSError, ET.ParseError, ValueError) as exc:
        sys.exit(f"[build_env_scene] Umgebung '{name}' ist kein lesbares MuJoCo-XML: {exc}")

    warnings = []
    mj, asset, wb = build_base(robot_file, f"g1_env_{name}")
    spawn = []
    try:
        n_obstacles, n_grasp = merge_environment(env_root, asset, wb, env_dir, warnings,
                                                 spawn_out=spawn)
    except Exception as exc:  # pragma: no cover - defensiv
        sys.exit(f"[build_env_scene] Umgebung '{name}' konnte nicht eingemischt "
                 f"werden: {exc}")
    if spawn:
        custom = ET.SubElement(mj, "custom")
        ET.SubElement(custom, "numeric", {"name": "g1_spawn", "data": _fmt(spawn[0])})

    ET.indent(mj, space="  ")
    header = (
        "<!-- AUTO-GENERIERT von scene_editor/build_env_scene.py.\n"
        f"     Basis (G1 + Licht + Boden + Weld) + Umgebung: {name}\n"
        f"     Roboter: {robot_file}\n"
        f"     Objekte: {n_obstacles} Hindernis(se), {n_grasp} Greif-Objekt(e)\n"
        "     NICHT von Hand editieren - wird bei jeder Umgebungs-Auswahl neu erzeugt. -->\n"
    )
    content = header + ET.tostring(mj, encoding="unicode") + "\n"

    # Atomar schreiben: erst als .tmp danebenlegen, testweise kompilieren, dann
    # umbenennen. So bleibt bei einem Fehler nie eine halbe/kaputte Szene liegen.
    tmp_path = out_path.with_name(out_path.name + ".tmp")
    try:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path.write_text(content, encoding="utf-8")
    except OSError as exc:
        sys.exit(f"[build_env_scene] Konnte '{out_path}' nicht schreiben: {exc}")

    if not args.no_validate:
        err = validate_scene(tmp_path)
        if err:
            tmp_path.unlink(missing_ok=True)
            for w in warnings:
                print(w, file=sys.stderr)
            sys.exit(f"[build_env_scene] Umgebung '{name}' laesst sich nicht laden:\n"
                     f"{err}\n"
                     "Tipp: Objekt-Namen/Meshes in der Umgebung pruefen "
                     "(scene_editor/README.md).")

    try:
        os.replace(tmp_path, out_path)
    except OSError as exc:
        tmp_path.unlink(missing_ok=True)
        sys.exit(f"[build_env_scene] Konnte '{out_path}' nicht ersetzen: {exc}")

    for w in warnings:
        print(w, file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()
