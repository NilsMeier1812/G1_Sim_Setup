#!/usr/bin/env python3
"""
Umgebungen als eigenstaendige Ordner: scenes/<name>/umgebung.xml + meshes/.

Eine Umgebung ist GENAU ein Ordner:

    scenes/<name>/
    ├── umgebung.xml          die Objekte (nur relative Pfade, siehe unten)
    └── meshes/               alle Dateien, die umgebung.xml benutzt
        ├── kiste.stl
        └── <cad-import>/     Einzelteile eines CAD-Imports + cad_import.json

Damit laesst sich eine Umgebung kopieren, zippen, loeschen und umbenennen,
ohne dass irgendwo etwas haengen bleibt. Der Ordnername ist der Name der
Umgebung (so heisst sie beim G1-Start / in G1_ENV).

Dieses Modul ist die EINE Stelle, die das Format kennt - Editor
(run_editor.py), Generator (build_env_scene.py), CAD-Import, launch.sh,
g1pilot/start.sh und die Start-GUI benutzen es. Nur Standardbibliothek, damit
es auch mit dem blanken System-python3 laeuft.

Uebergang: das alte Format (flache Datei scenes/<name>.xml, Meshes irgendwo
in meshes/) wird weiter gelesen; `migrate` wandelt es um.

CLI (launch.sh ruft das auf):
    python3 env_store.py list                     Namen, eine pro Zeile
    python3 env_store.py resolve <name|pfad>      Pfad der umgebung.xml
    python3 env_store.py check [name]             eigenstaendig? (Exit 1 = nein)
    python3 env_store.py migrate                  alte flache Umgebungen umwandeln
    python3 env_store.py pack <name> [ziel.zip]   Umgebung als Zip
    python3 env_store.py unpack <zip> [--name N] [--force]
"""
from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

HERE = Path(__file__).resolve().parent
SCENES_DIR = HERE / "scenes"
#: Mesh-Bibliothek (Uploads, Beispiele, CAD-Zwischenablage) - Quelle, aus der
#: beim Speichern in die Umgebung kopiert wird.
MESHES_DIR = HERE / "meshes"

ENV_FILE = "umgebung.xml"
ENV_MESH_DIR = "meshes"
#: Liegt in einem Mesh-Ordner eines CAD-Imports (Farben/Kollision je Teil).
CAD_MANIFEST = "cad_import.json"

#: Der Name wird Ordnername UND G1_ENV (docker-compose) - also streng.
_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")
#: Beifang alter Editor-Exporte, der sonst als Geister-Umgebung auftaucht.
_JUNK_NAMES = {"mujoco model", "mujoco_model"}
_UMLAUTS = {"ä": "ae", "ö": "oe", "ü": "ue", "ß": "ss", "Ä": "Ae", "Ö": "Oe", "Ü": "Ue"}

#: Asset-Attribute mit Dateipfaden (MJCF). Texturen koennen 6 Wuerfelseiten haben.
_FILE_ATTRS = {
    "mesh": ("file",),
    "skin": ("file",),
    "hfield": ("file",),
    "texture": ("file", "fileright", "fileleft", "fileup", "filedown",
                "filefront", "fileback"),
}
_DIR_ATTR = {"mesh": "meshdir", "skin": "meshdir", "hfield": "assetdir",
             "texture": "texturedir"}

#: Obergrenze beim Entpacken (Schutz vor kaputten/boesartigen Zips).
_MAX_UNPACKED = 4 * 1024 ** 3


def _log(msg: str) -> None:
    print(f"[env_store] {msg}", flush=True)


class EnvError(RuntimeError):
    """Fehler mit Klartext fuer Menschen."""


# ---------------------------------------------------------------------------
# Namen
# ---------------------------------------------------------------------------
def sanitize_env_name(raw: str) -> str:
    """Aus einer Nutzereingabe einen gueltigen Umgebungs-Namen machen ("" = unbrauchbar)."""
    name = Path((raw or "").strip()).name
    for k, v in _UMLAUTS.items():
        name = name.replace(k, v)
    for suffix in (".xml", ".json", ".zip"):
        if name.lower().endswith(suffix):
            name = name[: -len(suffix)]
    name = re.sub(r"\s+", "_", name)
    name = re.sub(r"[^A-Za-z0-9_\-]", "_", name)
    return re.sub(r"_{2,}", "_", name).strip("_-")


def env_dir(name: str) -> Path:
    return SCENES_DIR / name


def env_xml(name: str) -> Path:
    return env_dir(name) / ENV_FILE


def env_name_of(xml_path) -> str:
    """Name einer Umgebung aus dem Pfad ihrer XML (Ordner- bzw. Dateiname)."""
    p = Path(xml_path)
    return p.parent.name if p.name == ENV_FILE else p.stem


# ---------------------------------------------------------------------------
# Auflisten / Finden
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Env:
    name: str
    xml: Path
    legacy: bool = False            # altes Format: scenes/<name>.xml

    @property
    def folder(self) -> Path:
        return self.xml.parent


def _parse(path: Path) -> ET.Element:
    """MJCF einlesen, Kommentare bleiben erhalten (fuer das Zurueckschreiben)."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.fromstring(path.read_text(encoding="utf-8", errors="replace"), parser=parser)


def problem(xml: Path, name: str) -> str | None:
    """Warum das KEINE brauchbare Umgebung ist (sonst None)."""
    if name.strip().lower() in _JUNK_NAMES:
        return "Beifang eines alten Editor-Exports (kann geloescht werden)"
    if not _NAME_RE.match(name):
        return "Name enthaelt Leer-/Sonderzeichen (nur A-Z, a-z, 0-9, _ und -)"
    try:
        root = _parse(xml)
    except (OSError, ET.ParseError) as exc:
        return f"kein lesbares XML ({exc})"
    if root.tag != "mujoco":
        return f"Wurzel-Element <{root.tag}> statt <mujoco>"
    return None


def scan() -> tuple[list[Env], list[tuple[Path, str]]]:
    """-> (brauchbare Umgebungen, [(pfad, grund)] fuer unbrauchbare).

    Ordner-Format hat Vorrang: gibt es scenes/x/ UND scenes/x.xml, zaehlt der
    Ordner (die flache Datei ist dann ein Rest der Migration).
    """
    good, bad = {}, []
    if not SCENES_DIR.is_dir():
        return [], []
    for d in sorted(SCENES_DIR.iterdir()):
        if not d.is_dir() or d.name.startswith("."):
            continue
        xml = d / ENV_FILE
        if not xml.is_file():
            bad.append((d, f"Ordner ohne {ENV_FILE}"))
            continue
        why = problem(xml, d.name)
        if why:
            bad.append((d, why))
        else:
            good[d.name] = Env(d.name, xml)
    for f in sorted(SCENES_DIR.glob("*.xml")):
        if not f.is_file() or f.stem in good:
            continue
        why = problem(f, f.stem)
        if why:
            bad.append((f, why))
        else:
            good[f.stem] = Env(f.stem, f, legacy=True)
    return [good[k] for k in sorted(good, key=str.lower)], bad


def list_envs() -> list[Env]:
    return scan()[0]


def names() -> list[str]:
    return [e.name for e in list_envs()]


def find(name: str) -> Env | None:
    for e in list_envs():
        if e.name == name:
            return e
    return None


def resolve(arg: str) -> Path | None:
    """Umgebung finden: name | name.xml | scenes/name | .../umgebung.xml | Pfad.
    -> Pfad der XML oder None."""
    raw = Path(os.path.expanduser(str(arg)))
    cands = [raw] if raw.is_absolute() else [Path.cwd() / raw, HERE / raw]
    for c in cands:
        if c.is_dir() and (c / ENV_FILE).is_file():
            return (c / ENV_FILE).resolve()
        if c.is_file() and c.suffix.lower() == ".xml":
            return c.resolve()
    env = find(sanitize_env_name(raw.name) if raw.name != ENV_FILE else raw.parent.name)
    return env.xml.resolve() if env else None


# ---------------------------------------------------------------------------
# Eigenstaendigkeit: alle Dateien liegen im Umgebungsordner
# ---------------------------------------------------------------------------
def _compiler_dirs(root: ET.Element) -> dict:
    out = {}
    for comp in root.iter("compiler"):
        for attr in ("meshdir", "texturedir", "assetdir"):
            if comp.get(attr):
                out[attr] = comp.get(attr)
    return out


def file_refs(root: ET.Element, base: Path):
    """Alle Dateiverweise der Assets -> [(element, attribut, absoluter Pfad)].

    Relative Pfade gelten relativ zu `base` bzw. einem <compiler meshdir/...>.
    """
    dirs = _compiler_dirs(root)
    out = []
    for asset in root.iter("asset"):
        for el in asset:
            attrs = _FILE_ATTRS.get(el.tag)
            if not attrs:
                continue
            sub = dirs.get(_DIR_ATTR[el.tag]) or dirs.get("assetdir") or ""
            for attr in attrs:
                val = el.get(attr)
                if not val:
                    continue
                p = Path(os.path.expanduser(val.replace("\\", "/") if os.sep == "/" else val))
                if not p.is_absolute():
                    p = base / sub / p
                out.append((el, attr, p))
    return out


def _inside(path: Path, folder: Path) -> bool:
    try:
        path.resolve().relative_to(folder.resolve())
        return True
    except ValueError:
        return False


def containment_problems(xml: Path) -> list[str]:
    """Was die Umgebung von aussen braucht bzw. was fehlt ([] = eigenstaendig)."""
    try:
        root = _parse(xml)
    except (OSError, ET.ParseError) as exc:
        return [f"{xml.name}: nicht lesbar ({exc})"]
    folder = xml.parent
    out = []
    if _compiler_dirs(root):
        out.append("<compiler meshdir/texturedir/assetdir> gesetzt - Pfade bitte "
                   "direkt relativ zur umgebung.xml")
    for el, attr, p in file_refs(root, folder):
        raw = el.get(attr)
        if Path(raw).is_absolute() or re.match(r"^[A-Za-z]:[\\/]", raw):
            out.append(f"absoluter Pfad: {raw}")
        elif not _inside(p, folder):
            out.append(f"liegt ausserhalb des Umgebungsordners: {raw}")
        elif not p.is_file():
            out.append(f"Datei fehlt: {raw}")
    return out


def _same_file(a: Path, b: Path) -> bool:
    if a.stat().st_size != b.stat().st_size:
        return False
    ha, hb = hashlib.sha1(), hashlib.sha1()
    for p, h in ((a, ha), (b, hb)):
        with open(p, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    return ha.digest() == hb.digest()


def _free_path(dest: Path, src: Path) -> Path:
    """Zielpfad, der nicht mit einer ANDEREN Datei kollidiert: frei oder
    inhaltsgleich (dann wird die vorhandene Kopie mitbenutzt)."""
    cand, i = dest, 2
    while cand.exists() and not _same_file(cand, src):
        cand = dest.with_name(f"{dest.stem}_{i}{dest.suffix}")
        i += 1
    return cand


def make_self_contained(xml_in, folder, notes: list | None = None, prune: bool = False) -> Path:
    """Umgebung nach `folder`/umgebung.xml schreiben, alle Dateien hineinkopieren.

    xml_in darf ueberall liegen (z.B. frischer Editor-Export mit absoluten
    Pfaden). Dateien, die schon im Ordner liegen, bleiben, wo sie sind. Alle
    anderen kommen nach meshes/: Einzelteile eines CAD-Imports gemeinsam in
    einen Unterordner (mit cad_import.json, sonst kennt der Editor ihre Farben
    nicht mehr), sonstige Dateien direkt nach meshes/. Gleichnamige, aber
    verschiedene Dateien werden umbenannt statt ueberschrieben.

    prune=True: danach nicht mehr benutzte Dateien in meshes/ loeschen (beim
    Speichern im Editor - sonst sammeln sich Reste geloeschter Objekte).
    """
    notes = notes if notes is not None else []
    xml_in, folder = Path(xml_in).resolve(), Path(folder).resolve()
    root = _parse(xml_in)
    mesh_root = folder / ENV_MESH_DIR
    group_dirs: dict = {}       # CAD-Quellordner -> Zielordner
    used: set = set()
    copied = 0

    for el, attr, src in file_refs(root, xml_in.parent):
        src = src.resolve()
        if not src.is_file():
            # Relativer Pfad, der schon IM Zielordner gilt (z.B. Kopie einer
            # umgebung.xml ausserhalb ihres Ordners)? Dann gehoert die Datei
            # dazu - und darf beim Aufraeumen auf keinen Fall weg.
            here = (folder / el.get(attr)).resolve()
            if not Path(el.get(attr)).is_absolute() and here.is_file() and _inside(here, folder):
                used.add(here)
                continue
            notes.append(f"Datei fehlt: {el.get(attr)}")
            continue
        if _inside(src, folder):
            dest = src
        else:
            if (src.parent / CAD_MANIFEST).is_file():
                gdir = group_dirs.get(src.parent)
                if gdir is None:
                    manifest = src.parent / CAD_MANIFEST

                    def occupied(d: Path) -> bool:
                        # schon fuer einen anderen Import vergeben - oder ein
                        # vorhandener Ordner, der NICHT derselbe Import ist
                        if str(d).lower() in {str(v).lower() for v in group_dirs.values()}:
                            return True
                        if not d.exists():
                            return False
                        own = d / CAD_MANIFEST
                        return not (own.is_file() and _same_file(own, manifest))

                    gdir = mesh_root / src.parent.name
                    i = 2
                    while occupied(gdir):
                        gdir = mesh_root / f"{src.parent.name}_{i}"
                        i += 1
                    group_dirs[src.parent] = gdir
                    gdir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src.parent / CAD_MANIFEST, gdir / CAD_MANIFEST)
                dest = gdir / src.name
            else:
                dest = _free_path(mesh_root / src.name, src)
            if not (dest.exists() and _same_file(dest, src)):
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest)
                copied += 1
        used.add(dest.resolve())
        el.set(attr, os.path.relpath(dest, folder).replace(os.sep, "/"))

    for comp in root.iter("compiler"):          # Pfade sind jetzt direkt relativ
        for attr in ("meshdir", "texturedir", "assetdir"):
            comp.attrib.pop(attr, None)

    folder.mkdir(parents=True, exist_ok=True)
    out = folder / ENV_FILE
    ET.indent(root, space="  ")
    tmp = out.with_name(out.name + ".tmp")
    tmp.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")
    os.replace(tmp, out)
    if copied:
        notes.append(f"{copied} Datei(en) in den Umgebungsordner uebernommen")

    if prune and mesh_root.is_dir():
        removed = 0
        keep_dirs = {p.parent for p in used}
        for f in sorted(mesh_root.rglob("*"), reverse=True):
            if f.is_file():
                if f.resolve() in used:
                    continue
                if f.name == CAD_MANIFEST and f.parent.resolve() in keep_dirs:
                    continue
                f.unlink()
                removed += 1
            elif f.is_dir() and not any(f.iterdir()):
                f.rmdir()
        if removed:
            notes.append(f"{removed} nicht mehr benutzte Datei(en) entfernt")
    return out


# ---------------------------------------------------------------------------
# Migration altes -> neues Format
# ---------------------------------------------------------------------------
def migrate(env: Env, notes: list | None = None) -> Env:
    """scenes/<name>.xml (+ Meshes irgendwo) -> scenes/<name>/umgebung.xml + meshes/.

    Die Mesh-Bibliothek in meshes/ bleibt unangetastet (es wird kopiert); die
    alte flache Datei (und ihr .json vom Editor) wird erst entfernt, wenn der
    neue Ordner vollstaendig geschrieben ist.
    """
    notes = notes if notes is not None else []
    if not env.legacy:
        return env
    folder = env_dir(env.name)
    if (folder / ENV_FILE).exists():
        raise EnvError(f"{folder} gibt es schon - '{env.xml.name}' bitte von Hand pruefen.")
    tmp_folder = Path(tempfile.mkdtemp(prefix=f".{env.name}_", dir=SCENES_DIR))
    try:
        make_self_contained(env.xml, tmp_folder, notes)
        os.replace(tmp_folder, folder)
    except BaseException:
        shutil.rmtree(tmp_folder, ignore_errors=True)
        raise
    env.xml.unlink()
    sidecar = env.xml.with_suffix(".json")
    if sidecar.is_file():
        sidecar.unlink()          # Editor-Blueprint mit absoluten Pfaden - wertlos
    return Env(env.name, folder / ENV_FILE)


def migrate_all(log=_log) -> int:
    """Alle Umgebungen im alten Format umwandeln. -> Anzahl."""
    n = 0
    for env in list_envs():
        if not env.legacy:
            continue
        notes = []
        try:
            migrate(env, notes)
        except (EnvError, OSError) as exc:
            log(f"'{env.name}' nicht umgewandelt: {exc}")
            continue
        n += 1
        log(f"'{env.name}' -> scenes/{env.name}/{ENV_FILE}"
            + (f" ({'; '.join(notes)})" if notes else ""))
    return n


# ---------------------------------------------------------------------------
# Zip
# ---------------------------------------------------------------------------
def pack(name: str, out=None) -> Path:
    """Umgebung als Zip (Wurzelordner <name>/ darin). -> Pfad des Zips."""
    env = find(name)
    if env is None:
        raise EnvError(f"Umgebung '{name}' gibt es nicht.")
    if env.legacy:
        raise EnvError(f"'{name}' hat noch das alte Format - erst "
                       "'./launch.sh migrate' ausfuehren.")
    probs = containment_problems(env.xml)
    if probs:
        raise EnvError(f"'{name}' ist nicht eigenstaendig:\n  - " + "\n  - ".join(probs)
                       + "\n  Einmal im Editor oeffnen und speichern behebt das.")
    out = Path(out) if out else Path.cwd() / f"{name}.zip"
    if out.is_dir() or out.suffix.lower() != ".zip":      # Zielordner angegeben
        out = out / f"{name}.zip"
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + ".tmp")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for f in sorted(env.folder.rglob("*")):
            if f.is_file() and not f.name.endswith(".tmp") and "__pycache__" not in f.parts:
                zf.write(f, f"{name}/{f.relative_to(env.folder).as_posix()}")
    os.replace(tmp, out)
    return out


def _zip_layout(zf: zipfile.ZipFile) -> tuple[str, list]:
    """Pruefen, dass das Zip genau eine Umgebung ist. -> (praefix, eintraege)."""
    entries, total = [], 0
    for info in zf.infolist():
        name = info.filename.replace("\\", "/")
        if info.is_dir() or name.endswith("/"):
            continue
        parts = PurePosixPath(name).parts
        if name.startswith("/") or re.match(r"^[A-Za-z]:", name) or ".." in parts:
            raise EnvError(f"Unsicherer Pfad im Zip: {info.filename}")
        if parts and parts[0] == "__MACOSX":
            continue
        total += info.file_size
        entries.append((info, parts))
    if total > _MAX_UNPACKED:
        raise EnvError("Zip ist entpackt groesser als 4 GB - abgebrochen.")
    if any(p == (ENV_FILE,) for _i, p in entries):
        return "", entries                      # umgebung.xml direkt im Zip
    tops = {p[0] for _i, p in entries if len(p) > 1}
    if len(tops) == 1:
        top = tops.pop()
        if any(p == (top, ENV_FILE) for _i, p in entries):
            return top, entries
    raise EnvError(f"Kein {ENV_FILE} im Zip (erwartet: <name>/{ENV_FILE}).")


def unpack(zip_path, name: str | None = None, force: bool = False) -> Env:
    """Zip nach scenes/<name>/ entpacken und pruefen. -> Env."""
    zip_path = Path(zip_path)
    if not zip_path.is_file():
        raise EnvError(f"Datei nicht gefunden: {zip_path}")
    try:
        zf = zipfile.ZipFile(zip_path)
    except zipfile.BadZipFile as exc:
        raise EnvError(f"{zip_path.name} ist kein gueltiges Zip ({exc}).") from exc
    with zf:
        top, entries = _zip_layout(zf)
        name = sanitize_env_name(name or top or zip_path.stem)
        if not name:
            raise EnvError("Kein gueltiger Umgebungs-Name - bitte --name angeben.")
        folder = env_dir(name)
        if (folder.exists() or (SCENES_DIR / f"{name}.xml").exists()) and not force:
            raise EnvError(f"Umgebung '{name}' gibt es schon. Anderen Namen waehlen "
                           "(--name) oder ueberschreiben (--force).")
        SCENES_DIR.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=f".{name}_", dir=SCENES_DIR))
        try:
            for info, parts in entries:
                rel = parts[1:] if top else parts
                if not rel or (top and parts[0] != top):
                    continue
                dest = tmp.joinpath(*rel)
                if not _inside(dest, tmp):
                    raise EnvError(f"Unsicherer Pfad im Zip: {info.filename}")
                dest.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(info) as src, open(dest, "wb") as dst:
                    shutil.copyfileobj(src, dst)
            probs = containment_problems(tmp / ENV_FILE)
            if probs:
                raise EnvError("Die Umgebung im Zip ist nicht eigenstaendig:\n  - "
                               + "\n  - ".join(probs))
            if folder.exists():
                shutil.rmtree(folder)
            legacy = SCENES_DIR / f"{name}.xml"
            if legacy.is_file():
                legacy.unlink()
            os.replace(tmp, folder)
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
    return Env(name, folder / ENV_FILE)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Umgebungen (scenes/<name>/) verwalten")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="Namen aller Umgebungen")
    p = sub.add_parser("resolve", help="Pfad der umgebung.xml")
    p.add_argument("env")
    p = sub.add_parser("check", help="Umgebung(en) auf Eigenstaendigkeit pruefen")
    p.add_argument("env", nargs="?")
    p = sub.add_parser("migrate", help="altes Format (scenes/<name>.xml) umwandeln")
    p.add_argument("--quiet", action="store_true", help="nur melden, wenn etwas umgewandelt wurde")
    p = sub.add_parser("pack", help="Umgebung als Zip")
    p.add_argument("env")
    p.add_argument("out", nargs="?", help="Ziel (Datei oder Ordner, Default: ./<name>.zip)")
    p = sub.add_parser("unpack", help="Zip als Umgebung entpacken")
    p.add_argument("zip")
    p.add_argument("--name", help="anderer Name (Default: aus dem Zip)")
    p.add_argument("--force", action="store_true", help="vorhandene Umgebung ersetzen")
    args = ap.parse_args(argv)

    try:
        if args.cmd == "list":
            for n in names():
                print(n)
            return 0
        if args.cmd == "resolve":
            path = resolve(args.env)
            if path is None:
                print(f"Umgebung nicht gefunden: {args.env}", file=sys.stderr)
                return 1
            print(path)
            return 0
        if args.cmd == "check":
            envs = [find(args.env)] if args.env else list_envs()
            if args.env and envs[0] is None:
                print(f"Umgebung nicht gefunden: {args.env}", file=sys.stderr)
                return 1
            bad = 0
            for env in envs:
                probs = (["altes Format - './launch.sh migrate' ausfuehren"] if env.legacy
                         else containment_problems(env.xml))
                print(f"{'ok     ' if not probs else 'PROBLEM'}  {env.name}")
                for pr in probs:
                    print(f"         - {pr}")
                bad += bool(probs)
            return 1 if bad else 0
        if args.cmd == "migrate":
            n = migrate_all()
            if n:
                print(f"{n} Umgebung(en) ins Ordner-Format scenes/<name>/ umgewandelt.")
            elif not args.quiet:
                print("Nichts umzuwandeln.")
            return 0
        if args.cmd == "pack":
            out = pack(args.env, args.out)
            print(f"OK: {out} ({out.stat().st_size / 1e6:.1f} MB)")
            return 0
        if args.cmd == "unpack":
            env = unpack(args.zip, args.name, args.force)
            print(f"OK: Umgebung '{env.name}' -> {env.folder}")
            return 0
    except EnvError as exc:
        print(f"Fehler: {exc}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
