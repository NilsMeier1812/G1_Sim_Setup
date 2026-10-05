#!/usr/bin/env python
"""
Wrapper um die mujoco-scene-editor CLI.

Warum ueberhaupt ein Wrapper? Der Editor ist ein Fremdpaket; hier wird er auf
dieses Repo eingestellt und um genau das ergaenzt, was fuers Umgebungen-Bauen
fehlt - ohne das Paket zu patchen:

1. SPEICHERN NUR MIT NAMEN. Der eingebaute Export fragt nach einem kompletten
   DATEIPFAD ("~/temp/export/scene.json"). Das ist die haeufigste Fehlerquelle:
   ein Tippfehler im Pfad und die Umgebung landet irgendwo, wo sie niemand
   findet. Hier gibt es stattdessen oben den Ordner "Umgebung speichern" mit
   EINEM Feld: dem Namen. Gespeichert wird immer nach
   scene_editor/scenes/<name>/umgebung.xml .

2. JEDE UMGEBUNG EIN EIGENSTAENDIGER ORDNER. Beim Speichern werden alle
   benutzten Dateien (Meshes aus der Bibliothek, CAD-Importe, Dateien von
   ausserhalb des Repos) nach scenes/<name>/meshes/ kopiert und relativ
   referenziert, nicht mehr benutzte werden entfernt (env_store.py). Eine
   Umgebung laesst sich so als Ganzes kopieren, zippen und verteilen.

3. UMBENENNEN. Der eingebaute Editor kann Objekte nicht umbenennen - dabei
   entscheidet genau der Name, ob ein Objekt ein Hindernis oder ein GREIFBARES
   Objekt ist (Praefix "grasp_", siehe build_env_scene.py). Dafuer gibt es hier
   den Ordner "Objekt umbenennen" (mit Knopf "Greif-Objekt an/aus"). Derselbe
   Weg legt den Startpunkt des G1 fest: ein Objekt "g1_spawn" nennen.

4. UPLOAD-BUTTON + MESH-SKALIERUNG (echter Datei-Dialog bzw. Faktor fuer STLs).

5. ASSET-/MESH-Ordner auf  scene_editor/meshes/  vorbelegt (der eingebaute
   Default ~/temp/ArmarXObjects existiert nicht - dann wirkt der Ordner-Scan,
   als gaebe es keinen Import).

6. CAD-/MESH-IMPORT (STEP, IGES, BREP, PLY, GLB, ...). MuJoCo kann nur
   STL/OBJ/MSH. Alles andere laeuft durch cad_import.py und kommt als GRUPPE
   aus Einzelteilen in die Szene (Namen/Farben/Lage aus der Datei, konkave
   Teile konvex zerlegt) - ueber den Upload-Button oder den GUI-Ordner
   "CAD-Import" (Einstellungen + Dateien aus meshes/).

7. MESH-TUERSTEHER. Jedes Mesh wird geprueft, BEVOR es in die Szene kommt:
   ASCII-STL wird binaer neu geschrieben, zu grosse Netze (> 200000 Dreiecke)
   werden in Teile zerlegt statt abgelehnt. Sonst steckt ein unladbares Objekt
   in der Szene und auch das Speichern scheitert.

8. VIELE OBJEKTE. Eine CAD-Zelle hat schnell 700 Teile. Einfuegen passiert in
   einem Schritt, die Elementliste wird nur einmal aktualisiert, gleiche Meshes
   nur einmal geladen; Farbe/Kollision/Traegheit kommen beim Anzeigen und
   Speichern aus dem Import-Manifest (robits kennt dafuer keine Felder).

Aufruf wie die normale CLI:
    python run_editor.py new
    python run_editor.py edit scenes/environment_starter.xml
    python run_editor.py prompt "eine Kueche"
"""
import os
import socket
import sys
import tempfile
from pathlib import Path

# Haengender cachier-Eintrag (~/.cachier) darf den Editor nicht blockieren:
# Wird ein Editor beendet, WAEHREND er beim ersten Start den Objaverse-Katalog
# laedt, bleibt der Eintrag fuer immer als "in Berechnung" markiert. Jeder
# spaetere Start wartet dann endlos darauf -- die GUI erscheint zwar, aber die
# Knoepfe (z.B. "Add asset") sind nie verdrahtet und tun nichts. Nach 30 s
# Warten rechnet cachier jetzt selbst neu, statt ewig zu haengen.
import cachier
cachier.set_global_params(wait_for_calc_timeout=30)

HERE = Path(__file__).resolve().parent

# Umgebungen: scenes/<name>/umgebung.xml (+ meshes/), siehe env_store.py.
sys.path.insert(0, str(HERE))
import env_store  # noqa: E402

SCENES_DIR = env_store.SCENES_DIR
SCENES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_NAME = "meine_umgebung"
DEFAULT_TARGET = str(env_store.env_xml(DEFAULT_NAME))

# Ordner, aus dem der Editor eigene Meshes (STL/OBJ/...) importiert.
MESHES_DIR = HERE / "meshes"
MESHES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_ASSET_DIR = str(MESHES_DIR)

# Der Editor-Webserver (viser) haengt an diesem Port. Muss zu der URL passen,
# die g1_gui.py/launch.sh im Browser oeffnen.
EDITOR_PORT = int(os.environ.get("SCENE_EDITOR_PORT", "8080"))
os.environ.setdefault("_VISER_PORT_OVERRIDE", str(EDITOR_PORT))

# Der Editor bindet seinen Webserver fest an 127.0.0.1 (er darf Dateien
# schreiben - bewusst nur lokal erreichbar). Im Docker-Container ist 127.0.0.1
# aber das Container-Innere; dort setzt das Image SCENE_EDITOR_HOST=0.0.0.0,
# und docker-compose veroeffentlicht den Port trotzdem nur auf 127.0.0.1 des
# Rechners. Ohne die Variable bleibt alles wie gehabt.
EDITOR_HOST = os.environ.get("SCENE_EDITOR_HOST", "").strip()
if EDITOR_HOST:
    import viser as _viser

    _orig_viser_init = _viser.ViserServer.__init__

    def _viser_init_with_host(self, host="127.0.0.1", *args, **kwargs):
        _orig_viser_init(self, EDITOR_HOST, *args, **kwargs)

    _viser.ViserServer.__init__ = _viser_init_with_host

# Gemeinsame Helfer mit dem Szenen-Generator (liegt im selben Ordner).
sys.path.insert(0, str(HERE))
import build_env_scene as bes  # noqa: E402


def _port_in_use(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
        except OSError:
            return True
    return False


if _port_in_use(EDITOR_PORT):
    sys.exit(
        f"[run_editor] Port {EDITOR_PORT} ist schon belegt - vermutlich laeuft der Editor\n"
        f"             bereits (Browser: http://127.0.0.1:{EDITOR_PORT}).\n"
        f"             In der GUI: 'Laufende Prozesse' -> Stoppen. Oder einen anderen\n"
        f"             Port nehmen: SCENE_EDITOR_PORT=8081 ./launch.sh ...")


# Die vorbelegten Pfade in allen Modulen setzen, die sie beim Aufbauen der GUI
# lesen. Wir patchen die schon importierten Modul-Globals, damit es unabhaengig
# von der Importreihenfolge wirkt.
import mujoco_scene_editor.constants as _constants  # noqa: E402
_constants.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_constants.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.layout as _layout  # noqa: E402
_layout.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_layout.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.cli.editor_cli as _editor_cli  # noqa: E402
_editor_cli.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET

import mujoco_scene_editor.scene_editor as _scene_editor_mod  # noqa: E402

# robits meldet beim Einlesen JEDES Meshes pauschal "Not fully implemented
# yet." - bei einer CAD-Zelle 750 Zeilen, zwischen denen echte Warnungen
# ("Mesh path does not exist.") untergehen. Nur genau diese Meldung weg.
import logging  # noqa: E402


class _DropMeshImportNoise(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return record.getMessage() != "Not fully implemented yet."


logging.getLogger("robits.sim.converters.mujoco_importer").addFilter(_DropMeshImportNoise())


# ---------------------------------------------------------------------------
# Ohne Internet ueberhaupt starten koennen.
# Der Editor laedt beim Start den Objaverse-Online-Katalog (Objekt-Bibliothek).
# Faellt der aus (kein/gesperrtes Internet, HTTP 403), stirbt sonst der GANZE
# Editor - obwohl man ihn zum Bauen eigener Umgebungen gar nicht braucht.
# ---------------------------------------------------------------------------
_orig_load_inventory = _scene_editor_mod.SceneEditor._load_inventory


def _load_inventory_offline_tolerant(self):
    try:
        _orig_load_inventory(self)
        return
    except Exception as exc:
        print(f"[run_editor] Objekt-Katalog (Objaverse) nicht erreichbar: {exc}\n"
              "             Editor laeuft weiter - eigene Shapes/STLs gehen normal, "
              "nur die Online-Bibliothek bleibt leer.", file=sys.stderr)
    # Lokale Meshes trotzdem anbieten, Online-Liste leer lassen.
    try:
        items = self.inventory.list(
            root=Path(self.layout.assets_dir.value.strip()).expanduser())
        self.layout.asset_items = {m.name: m for m in items}
    except Exception:
        self.layout.asset_items = {}
    self.layout.update_assets_dropdown()
    self.layout.objaverse_labels = ()
    self.layout.update_objaverse_label_dropdown()
    self.layout.objaverse_items = {}
    self.layout.update_objaverse_dropdown()


_scene_editor_mod.SceneEditor._load_inventory = _load_inventory_offline_tolerant


# ---------------------------------------------------------------------------
# Namen
# ---------------------------------------------------------------------------
# Der Name wird Ordnername UND Wert von G1_ENV (docker-compose) - die Regeln
# stehen an EINER Stelle (env_store).
sanitize_env_name = env_store.sanitize_env_name


def _initial_name_from_argv() -> str:
    """Beim Bearbeiten einer vorhandenen Umgebung deren Namen vorbelegen."""
    argv = sys.argv[1:]
    if len(argv) >= 2 and argv[0] == "edit":
        name = sanitize_env_name(env_store.env_name_of(argv[1]))
        if name:
            return name
    return DEFAULT_NAME


INITIAL_NAME = _initial_name_from_argv()


# ---------------------------------------------------------------------------
# Speichern (Export) - nur der Name wird gefragt
# ---------------------------------------------------------------------------
def _repair_orphan_parents(editor, notes: list) -> None:
    """Verwaiste Eltern-Pfade reparieren, BEVOR exportiert wird.

    Hintergrund (Bug im Fremdpaket 'robits', das der Editor fuers Exportieren
    nutzt): neue Objekte bekommen als Eltern-Pfad das, was gerade oben unter
    "Elements" ausgewaehlt ist. Ist dabei ein normales Objekt (Box/Mesh/...)
    statt eine ECHTE Gruppe ausgewaehlt, haengt der Editor das neue Objekt
    intern unter diesen Pfad (z.B. "/box_0001/box_0002"). Der MuJoCo-Exporter
    traegt aber NUR Gruppen (BlueprintGroup) in seine interne Eltern-Zuordnung
    ein -- fuer alles andere schlaegt die Suche fehl und der Export stuerzt mit
    "AttributeError: 'NoneType' object has no attribute 'joints'" ab (robits/
    sim/scene/mjcf_utils.py:has_freejoint auf einem nicht gefundenen Parent).

    Wir heben solche Objekte hier auf die oberste Ebene, statt den Export
    scheitern zu lassen - fuer unsere Umgebungen (nur Hindernisse/Greif-
    objekte, siehe scene_editor/README.md) macht flach vs. verschachtelt
    ohnehin keinen Unterschied: build_env_scene.py flacht beim Kombinieren
    sowieso alles ab.

    Die Pose eines solchen Kindes ist RELATIV zum Eltern-Objekt (der Editor
    zeichnet es im viser-Szenengraphen darunter). Beim Hochheben wird sie
    darum mit allen vorhandenen Vorfahren verrechnet (Welt-Pose) - sonst
    springt das Objekt beim Speichern an eine andere Stelle.
    """
    from dataclasses import replace as _dc_replace

    import numpy as np
    from robits.sim.blueprints import BlueprintGroup, Pose

    ctrl = editor.controller
    blueprints = ctrl.state.blueprints

    def world_matrix(path, bp):
        m = np.asarray(bp.pose.matrix)
        parent = path.rsplit("/", 1)[0]
        while parent:
            pbp = blueprints.get(parent)
            if pbp is not None:
                m = np.asarray(pbp.pose.matrix) @ m
            parent = parent.rsplit("/", 1)[0]
        return m
    top_level_groups = {
        p.strip("/").split("/", 1)[0]
        for p, bp in blueprints.items()
        if isinstance(bp, BlueprintGroup)
    }
    used_top_names = {p.strip("/").split("/", 1)[0] for p in blueprints}

    fixed = {}
    for path, bp in blueprints.items():
        parts = path.strip("/").split("/")
        if len(parts) <= 1 or parts[0] in top_level_groups:
            continue
        leaf = parts[-1] or "objekt"
        new_path = f"/{leaf}"
        n = 2
        while new_path in blueprints or new_path in fixed or \
                new_path.strip("/") in used_top_names:
            new_path = f"/{leaf}_{n}"
            n += 1
        fixed[new_path] = _dc_replace(bp, path=new_path,
                                      pose=Pose(matrix=world_matrix(path, bp)))
        used_top_names.add(new_path.strip("/"))
        notes.append(f"'{path}' hatte keinen gueltigen Eltern-Pfad mehr "
                     f"-> auf oberste Ebene gehoben als '{new_path}'.")

    if not fixed:
        return

    ctrl.state.push_state_to_history()
    for old_path in list(blueprints):
        parts = old_path.strip("/").split("/")
        if len(parts) > 1 and parts[0] not in top_level_groups:
            del blueprints[old_path]
    blueprints.update(fixed)
    ctrl.renderer.render_from_state(list(blueprints.values()))


def _fix_bad_stl_meshes(editor, notes: list) -> None:
    """STL-Meshes reparieren, BEVOR robits die Szene zum Exportieren kompiliert.

    MuJoCo laedt nur BINAERE STL-Dateien. ASCII-STL (haeufig bei Assets aus
    Blender/Sketchfab/Objaverse-Downloads) laesst robits' Kompilierschritt mit
    "ValueError: ... stl_decoder: ... perhaps this is an ASCII file?" abstuerzen
    - noch bevor build_env_scene.py ueberhaupt ins Spiel kommt. Alle im
    aktuellen Editor-Stand referenzierten STLs werden deshalb hier schon
    geprueft und bei Bedarf binaer neu geschrieben (mit trimesh, das der Editor
    ohnehin als Abhaengigkeit mitbringt).
    """
    from robits.sim.blueprints import MeshBlueprint

    for bp in editor.controller.state.blueprints.values():
        if not isinstance(bp, MeshBlueprint):
            continue
        mesh_path = Path(bp.mesh_path)
        if mesh_path.suffix.lower() != ".stl" or not mesh_path.is_file():
            continue
        if bes.is_valid_binary_stl(mesh_path):
            continue
        bes.convert_stl_to_binary(mesh_path, notes)


def save_environment(editor, name: str):
    """Szene als eigenstaendigen Ordner scenes/<name>/ speichern.

    Rueckgabe: (ok, titel, text). Exportiert wird IMMER erst in einen
    temporaeren Ordner (der Exporter legt dort noch "MuJoCo Model.xml" und eine
    .json mit absoluten Pfaden ab - beides gehoert nicht in die Umgebung).
    env_store.make_self_contained() schreibt dann umgebung.xml, holt alle
    benutzten Dateien in den Ordner und raeumt nicht mehr benutzte weg.
    """
    clean = sanitize_env_name(name)
    if not clean:
        return False, "Name fehlt", (
            "Bitte einen Namen eingeben (Buchstaben, Ziffern, _ und -), "
            "z.B. 'kueche'.")

    folder = env_store.env_dir(clean)
    target = folder / env_store.ENV_FILE
    legacy = SCENES_DIR / f"{clean}.xml"
    existed = target.is_file() or legacy.is_file()
    notes = []

    try:
        _repair_orphan_parents(editor, notes)
        _fix_bad_stl_meshes(editor, notes)
        with tempfile.TemporaryDirectory(prefix="scene_export_") as td:
            tmp_xml = Path(td) / "export.xml"
            editor.controller.export_scene(tmp_xml)
            if not tmp_xml.is_file():
                return False, "Speichern fehlgeschlagen", \
                    "Der Editor hat keine XML-Datei erzeugt."
            env_store.make_self_contained(tmp_xml, folder, notes, prune=True)
        # Dieselbe Umgebung im alten Format (scenes/<name>.xml) abloesen.
        for old in (legacy, legacy.with_suffix(".json")):
            if old.is_file():
                old.unlink()
    except Exception as exc:  # pragma: no cover - Laufzeit
        return False, "Speichern fehlgeschlagen", f"{type(exc).__name__}: {exc}"

    # Kontrolle: vollstaendig und ladbar?
    problems = env_store.containment_problems(target)
    problem = bes.validate_scene(target)

    lines = [f"{'Ueberschrieben' if existed else 'Gespeichert'}: "
             f"scenes/{clean}/{env_store.ENV_FILE}"]
    lines.append("Beim Sim-Start unter 'Umgebung' als '%s' waehlbar." % clean)
    if notes:
        lines.append("Hinweise: " + "; ".join(notes))
    if problems:
        lines.append("ACHTUNG, nicht eigenstaendig: " + "; ".join(problems))
    if problem:
        lines.append("ACHTUNG: MuJoCo kann die Datei nicht laden -> " + problem.strip())
    if problems or problem:
        return False, "Gespeichert, aber fehlerhaft", "\n".join(lines)
    return True, "Umgebung gespeichert", "\n".join(lines)


def _install_save_control(editor) -> None:
    """Ordner "Umgebung speichern" - EIN Feld: der Name."""
    server = editor.layout.server
    try:
        with server.gui.add_folder("Umgebung speichern", order=1.1,
                                   expand_by_default=True):
            txt = server.gui.add_text(
                "Name", initial_value=INITIAL_NAME,
                hint="Nur der Name, kein Pfad. Gespeichert wird immer als "
                     "Ordner scene_editor/scenes/<name>/")
            btn = server.gui.add_button("Speichern", color="green")
            server.gui.add_markdown(
                "Ziel: `scene_editor/scenes/<name>/` (umgebung.xml + alle Meshes) — "
                "genau diese Namen stehen beim Sim-Start unter *Umgebung* zur Auswahl.")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Speichern-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    # Der eingebaute Export-Pfad wird ausgeblendet (er verwirrt nur), bleibt aber
    # als Handle bestehen: "Launch MuJoCo" liest ihn aus.
    for handle in (editor.layout.export_path, editor.layout.btn_export_mj):
        try:
            handle.visible = False
        except Exception:
            pass
    editor.layout.export_path.value = str(env_store.env_xml(INITIAL_NAME))

    @btn.on_click
    def _save(event) -> None:
        btn.disabled = True
        try:
            ok, title, body = save_environment(editor, txt.value)
            clean = sanitize_env_name(txt.value)
            if clean:
                txt.value = clean
                editor.layout.export_path.value = str(env_store.env_xml(clean))
            print(f"[run_editor] {title}: {body.replace(chr(10), ' | ')}")
            _notify(event, title, body)
        finally:
            btn.disabled = False


# ---------------------------------------------------------------------------
# Umbenennen (macht die grasp_-Konvention ueberhaupt erst benutzbar)
# ---------------------------------------------------------------------------
def _install_rename_control(editor) -> None:
    from dataclasses import replace as _dc_replace

    from mujoco_scene_editor.constants import NO_SELECTION

    server = editor.layout.server
    ctrl = editor.controller
    try:
        with server.gui.add_folder("Objekt umbenennen", order=1.2,
                                   expand_by_default=True):
            txt = server.gui.add_text(
                "Neuer Name", initial_value="",
                hint="Oben unter 'Elements' ein Objekt waehlen. Praefix "
                     "'grasp_' = greifbares Objekt (beweglich), sonst Hindernis.")
            btn = server.gui.add_button("Umbenennen")
            # Ein Klick statt Tippen: Praefix grasp_ setzen/entfernen.
            btn_grasp = server.gui.add_button("Greif-Objekt an/aus (grasp_)")
            server.gui.add_markdown(
                "Startpunkt des G1: ein Objekt `g1_spawn` nennen (siehe README).")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Umbenennen-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @editor.layout.element_list.on_update
    def _sync_name(_evt) -> None:
        sel = editor.layout.element_list.value
        if sel and sel != NO_SELECTION:
            txt.value = sel.rsplit("/", 1)[-1]

    def _selected(event):
        old = editor.layout.element_list.value
        if not old or old == NO_SELECTION or old not in ctrl.state.blueprints:
            _notify(event, "Kein Objekt gewaehlt",
                    "Bitte oben unter 'Elements' ein Objekt auswaehlen.")
            return None
        return old

    @btn.on_click
    def _rename(event) -> None:
        old = _selected(event)
        if old is not None:
            _apply(event, old, txt.value)

    @btn_grasp.on_click
    def _toggle_grasp(event) -> None:
        old = _selected(event)
        if old is None:
            return
        leaf = old.rsplit("/", 1)[-1]
        prefix = "grasp_"
        _apply(event, old, leaf[len(prefix):] if bes.is_grasp_name(leaf) else prefix + leaf)

    def _apply(event, old: str, raw_name: str) -> None:
        new_leaf = bes.sanitize_name(raw_name)
        if not new_leaf:
            _notify(event, "Name ungueltig",
                    "Erlaubt sind Buchstaben, Ziffern, _ und - (keine Slashes).")
            return
        parent = old.rsplit("/", 1)[0]
        new = f"{parent}/{new_leaf}"
        if new == old:
            return
        if new in ctrl.state.blueprints:
            _notify(event, "Name schon vergeben",
                    f"'{new_leaf}' gibt es in dieser Ebene bereits.")
            return

        try:
            ctrl.state.push_state_to_history()
            renamed = {}
            for path, bp in ctrl.state.blueprints.items():
                if path == old:
                    renamed[new] = _dc_replace(bp, path=new)
                elif path.startswith(old + "/"):
                    child = new + path[len(old):]
                    renamed[child] = _dc_replace(bp, path=child)
                else:
                    renamed[path] = bp
            ctrl.state.blueprints = renamed
            ctrl.renderer.render_from_state(list(ctrl.state.blueprints.values()))
            editor.layout.element_list.value = new
            ctrl.select(new)
        except Exception as exc:  # pragma: no cover - Laufzeit
            _notify(event, "Umbenennen fehlgeschlagen", f"{type(exc).__name__}: {exc}")
            return
        txt.value = new_leaf
        kind = "greifbares Objekt" if bes.is_grasp_name(new_leaf) else "Hindernis"
        _notify(event, "Umbenannt", f"{old.rsplit('/', 1)[-1]} -> {new_leaf}  ({kind})")


# CAD-/Mesh-Import (STEP, IGES, PLY, GLB, zu grosse STL, ...). Liegt neben
# diesem Skript; OpenCascade/trimesh werden erst beim Import geladen.
import cad_import  # noqa: E402
import mesh_utils  # noqa: E402

# ---------------------------------------------------------------------------
# Asset-Scan ohne Cache. Der Editor cached die Ordner-Liste per cachier 30 Tage
# lang in ~/.cachier -- neu nach meshes/ kopierte STLs fehlen dann im Dropdown
# "Items", obwohl sie im Ordner liegen. Der Scan ist ein simples rglob und
# braucht keinen Cache, also immer frisch scannen. Versteckte Ordner (z.B.
# meshes/.collision/ mit den Kollisions-Teilen von build_env_scene.py) bleiben
# aussen vor - sonst stuenden dort Hunderte hull_XX.stl in der Liste.
# ---------------------------------------------------------------------------
import mujoco_scene_editor.inventory.local_assets as _local_assets  # noqa: E402


def _list_assets_uncached(self, root, *_args, **_cachier_kwargs):
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        return []
    items = [
        _local_assets.ObjectModel(name=p.stem, path=p)
        for p in _local_assets._iter_asset_files(root, _local_assets.ASSET_EXTS)
        if not any(part.startswith(".") for part in p.relative_to(root).parts)
    ]
    items.sort(key=lambda m: m.name.lower())
    return items


_local_assets.Inventory.list = _list_assets_uncached


# ---------------------------------------------------------------------------
# Zusaetzlicher Upload-Button: echter Datei-Dialog des Browsers.
# Der eingebaute Import ("Add Assets from File") scannt nur einen Ordner. Fuer
# "Datei aus beliebigem Ordner auswaehlen" haengen wir per viser-Upload-Button
# einen zweiten Weg an. Umgesetzt ohne Aenderung am Fremdpaket, indem wir die
# Editor-Fabrik umschliessen.
#
#  * STL/OBJ, die MuJoCo direkt laden kann -> wie bisher ein Mesh-Objekt.
#  * alles andere (STEP/IGES/BREP, PLY/GLB/..., ASCII-/Riesen-STL) laeuft durch
#    cad_import.py und kommt als GRUPPE aus Einzelteilen in die Szene - mit
#    Namen, Farben und Lage aus der Datei.
# ---------------------------------------------------------------------------
# Mesh-Uploads landen wie bisher direkt in meshes/ (der Ordner-Scan findet sie),
# CAD-Quellen daneben in meshes/uploads/ (der Scan kennt die Endungen nicht).
UPLOADS_DIR = MESHES_DIR / "uploads"

# Einstellungen fuer den CAD-Import (im GUI-Ordner "CAD-Import" aenderbar)
_CAD_OPTS = {
    "quality": cad_import.DEFAULT_QUALITY,
    "scale": 0.0,                  # 0 = automatisch
    "place": cad_import.DEFAULT_PLACEMENT,
    "min_collision_size": cad_import.DEFAULT_MIN_COLLISION_SIZE,
}


def _cad_options() -> "cad_import.ImportOptions":
    scale = float(_CAD_OPTS["scale"] or 0.0)
    return cad_import.ImportOptions(
        quality=str(_CAD_OPTS["quality"]),
        scale=scale if scale > 0 else None,
        place=str(_CAD_OPTS["place"]),
        min_collision_size=float(_CAD_OPTS["min_collision_size"]))


def _upload_suffixes() -> str:
    """Endungen fuer den Datei-Dialog (Browser filtern exakt: auch GROSS)."""
    exts = cad_import.supported_suffixes()
    return ",".join(exts + [e.upper() for e in exts])


def _notify(event, title, body, loading=False):
    try:
        return event.client.add_notification(
            title=title, body=body, loading=loading, with_close_button=not loading)
    except Exception:
        print(f"[run_editor] {title}: {body}")
        return None


def _broadcast(editor, title, body) -> None:
    """Meldung an alle offenen Browser-Tabs (fuer Stellen ohne GUI-Event)."""
    try:
        for client in editor.layout.server.get_clients().values():
            client.add_notification(title=title, body=body, loading=False)
    except Exception:
        pass


def _done(handle) -> None:
    """Lade-Meldung wegnehmen (viser: nur remove() ist oeffentlich)."""
    try:
        if handle is not None:
            handle.remove()
    except Exception:
        pass


# -- Schnelles Einfuegen/Neuzeichnen vieler Objekte --------------------------
# Der Renderer schickt bei JEDEM Element die komplette Elementliste an den
# Browser. Bei einer CAD-Zelle mit ~700 Teilen waeren das ~500.000 Eintraege.
# Waehrend Sammel-Operationen wird das deshalb angehalten und am Ende EINMAL
# nachgeholt.
def _batched_dropdown(renderer):
    class _Ctx:
        def __enter__(self):
            self.orig = renderer.update_elements_dropdown
            renderer.update_elements_dropdown = lambda *_a, **_k: None
            return self

        def __exit__(self, *exc):
            renderer.update_elements_dropdown = self.orig
            self.orig()
            return False
    return _Ctx()


def _patch_renderer_for_many_objects() -> None:
    from mujoco_scene_editor.scene_renderer import ViserSceneRenderer

    orig = ViserSceneRenderer.render_from_state

    def render_from_state(self, blueprints):
        with _batched_dropdown(self):
            return orig(self, blueprints)

    ViserSceneRenderer.render_from_state = render_from_state


def _patch_mesh_rendering() -> None:
    """Meshes mit Farbe aus dem CAD-Manifest zeichnen, Kollisions-Stuecke
    ausblenden und gleiche Dateien nur einmal laden.

    Der eingebaute Renderer kennt fuer Meshes keine Farbe (alles grau) und
    laedt jede Datei neu - bei 700 Teilen aus 300 Dateien unnoetig langsam.
    """
    import trimesh
    from mujoco_scene_editor.scene_renderer import ViserSceneRenderer
    from mujoco_scene_editor.utils import viser_utils
    from robits.sim.blueprints import MeshBlueprint

    cache = {}

    def _load(path: Path):
        key = (str(path), path.stat().st_mtime)
        tri = cache.get(key)
        if tri is None:
            tri = trimesh.load_mesh(str(path))
            cache[key] = tri
        return tri.copy()

    def _create_mesh_node(self, bp):
        position, wxyz = viser_utils.pose_to_gui(bp)
        path = Path(bp.mesh_path).resolve()
        tri = _load(path)
        scale = getattr(bp, "scale", None)
        if scale is not None and scale != 1.0:
            tri.apply_scale(scale)
        defaults = cad_import.mesh_defaults(path) or {}
        if defaults.get("rgba"):
            rgba = [int(round(255 * c)) for c in defaults["rgba"]]
            tri.visual = trimesh.visual.ColorVisuals(tri, face_colors=rgba)
        return self.layout.server.scene.add_mesh_trimesh(
            bp.path, mesh=tri, wxyz=wxyz, position=position,
            visible=defaults.get("visual", True))

    ViserSceneRenderer.__dict__["_create_node"].register(MeshBlueprint, _create_mesh_node)


def _patch_mesh_export() -> None:
    """Beim Speichern Farbe/Kollision/Traegheit aus dem CAD-Manifest setzen.

    robits exportiert Meshes ohne Farbe, immer mit Kollision und kompiliert
    die Szene dabei - flache/offene CAD-Teile (Bleche, Schilder) liessen das
    mit "mesh volume is too small" scheitern. Die Werte stehen im Manifest des
    Imports (cad_import.mesh_defaults).
    """
    import mujoco
    from robits.sim.blueprints import MeshBlueprint
    from robits.sim.scene.model_factory import SceneBuilder

    dispatcher = SceneBuilder.__dict__["add"]
    orig = dispatcher.dispatcher.dispatch(MeshBlueprint)

    def add_mesh(self, blueprint):
        out = orig(self, blueprint)
        defaults = cad_import.mesh_defaults(blueprint.mesh_path)
        if not defaults:
            return out
        try:
            mesh = self.spec.mesh(f"{blueprint.basename}_mesh")
            geom = self.spec.geom(blueprint.basename)
        except Exception:
            return out
        if mesh is not None and defaults.get("shell"):
            mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
        if geom is not None:
            geom.rgba = list(defaults.get("rgba", geom.rgba))
            if not defaults.get("collision", True):
                geom.contype = 0
                geom.conaffinity = 0
            if not defaults.get("visual", True):
                geom.group = int(cad_import.COLLISION_GROUP)
        return out

    dispatcher.register(MeshBlueprint, add_mesh)


def _hide_cad_parts_in_asset_scan() -> None:
    """Einzelteile aus meshes/cad/ nicht im Ordner-Scan anbieten.

    Der Scan laeuft rekursiv - ohne Filter stuenden nach einem Zellen-Import
    Hunderte Einzelteile in "Add Assets from File". Ganze Importe fuegt man
    ueber den Ordner "CAD-Import" ein.
    """
    from mujoco_scene_editor.inventory.local_assets import Inventory

    orig = Inventory.list

    def list_without_cad(self, root, *args, **kwargs):
        items = orig(self, root, *args, **kwargs)
        cad_root = cad_import.CAD_DIR.resolve()
        return [m for m in items if cad_root not in Path(m.path).resolve().parents]

    Inventory.list = list_without_cad


def insert_cad_import(editor, result, place: str) -> str:
    """Import-Ergebnis als Gruppe in die Szene legen. -> Pfad der Gruppe.

    Alle Blueprints kommen in EINEM Schritt in den Zustand (ein Undo-Schritt,
    eine Dropdown-Aktualisierung). Teile mit konvexer Zerlegung bekommen eine
    eigene Untergruppe, damit sichtbares Mesh und Kollisions-Stuecke beim
    Verschieben/Loeschen zusammenbleiben. Namen werden gegen die ganze Szene
    eindeutig gemacht (MuJoCo verlangt global eindeutige Namen).
    """
    import numpy as np
    from robits.sim.blueprints import BlueprintGroup, MeshBlueprint, Pose

    ctrl = editor.controller
    used = {p.rsplit("/", 1)[-1].lower() for p in ctrl.state.blueprints}

    def unique(name):
        return cad_import._unique(bes.sanitize_name(name) or "teil", used)

    def pose(pos, quat=(1.0, 0.0, 0.0, 0.0)):
        return Pose().with_position([float(v) for v in pos]).with_quat_wxyz(
            [float(v) for v in quat])

    root = "/" + unique(result.name)
    bps = [BlueprintGroup(root, pose(result.placement_offset(place)))]
    hulls = {}
    for inst in result.instances:
        if inst.part:
            hulls.setdefault(inst.part, []).append(inst)
    for inst in result.parts:
        mesh = str((result.out_dir / inst.mesh).resolve())
        leaf = unique(inst.name)
        if inst.name not in hulls:
            bps.append(MeshBlueprint(f"{root}/{leaf}", mesh_path=mesh,
                                     pose=pose(inst.pos, inst.quat), is_static=True))
            continue
        group = f"{root}/{leaf}"
        bps.append(BlueprintGroup(group, pose(inst.pos, inst.quat)))
        bps.append(MeshBlueprint(f"{group}/{unique(inst.name + '_optik')}", mesh_path=mesh,
                                 pose=pose((0, 0, 0)), is_static=True))
        R = np.asarray(Pose().with_quat_wxyz(inst.quat).matrix)[:3, :3]
        for h in hulls[inst.name]:
            rel = R.T @ (np.asarray(h.pos) - np.asarray(inst.pos))
            bps.append(MeshBlueprint(
                f"{group}/{unique(h.name)}", mesh_path=str((result.out_dir / h.mesh).resolve()),
                pose=pose(rel), is_static=True))

    ctrl.state.push_state_to_history()
    for bp in bps:
        ctrl.state.blueprints[bp.path] = bp
    ctrl.state._seq += 1
    with _batched_dropdown(ctrl.renderer):
        for bp in bps:
            ctrl.renderer.add(bp)
    ctrl.update_history_btn_visibility()
    return root


def _import_and_insert(editor, src: Path, event=None, name=None) -> None:
    """Datei per cad_import einlesen und in die Szene legen (mit Meldungen)."""
    options = _cad_options()
    busy = _notify(event, "Import laeuft ...",
                   f"{src.name} wird eingelesen - grosse Baugruppen brauchen ein "
                   "paar Minuten (Fortschritt im Terminal).", loading=True) \
        if event is not None else None
    try:
        result = cad_import.import_file(src, name=name, options=options)
        group = insert_cad_import(editor, result, options.place)
    except cad_import.ImportFailed as exc:
        _done(busy)
        print(f"[run_editor] Import fehlgeschlagen: {exc}", file=sys.stderr)
        (_notify(event, "Import fehlgeschlagen", str(exc)) if event is not None
         else _broadcast(editor, "Import fehlgeschlagen", str(exc)))
        return
    except Exception as exc:  # pragma: no cover - Laufzeit
        _done(busy)
        msg = f"{type(exc).__name__}: {exc}"
        print(f"[run_editor] Import fehlgeschlagen: {msg}", file=sys.stderr)
        (_notify(event, "Import fehlgeschlagen", msg) if event is not None
         else _broadcast(editor, "Import fehlgeschlagen", msg))
        return
    _done(busy)
    body = (f"{result.summary()}.\nIn der Szene als Gruppe '{group.strip('/')}' - "
            "die ganze Gruppe unter 'Elements' waehlen, um sie zu verschieben.")
    if result.notes:
        body += "\nHinweise: " + " ".join(result.notes[:4])
    print(f"[run_editor] {body}")
    if event is not None:
        _notify(event, "CAD-Import eingefuegt", body)
    else:
        _broadcast(editor, "CAD-Import eingefuegt", body)


def _install_upload_button(editor) -> None:
    server = editor.layout.server
    cad_ok = cad_import.occ_available()
    label = "Datei waehlen (STL/OBJ/STEP/...)" if cad_ok else "Datei waehlen (STL/OBJ/...)"
    hint = ("Datei aus beliebigem Ordner. STL/OBJ kommen direkt in die Szene; "
            "CAD (STEP/IGES) und andere Formate werden in Einzelteile zerlegt "
            "(Einstellungen im Ordner 'CAD-Import').")
    try:
        with server.gui.add_folder("Eigene Datei hochladen", order=1.3,
                                   expand_by_default=True):
            up = server.gui.add_upload_button(label, mime_type=_upload_suffixes(), hint=hint)
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Upload-Button nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @up.on_upload
    def _on_upload(event) -> None:
        f = up.value
        if not f or not f.name:
            return
        name = Path(f.name).name
        if cad_import.is_cad_file(name) and not cad_ok:
            _notify(event, "CAD-Import nicht moeglich", cad_import.NO_OCC_HINT)
            return
        simple = Path(name).suffix.lower() in mesh_utils.MJ_MESH_SUFFIXES
        dest = (MESHES_DIR if simple else UPLOADS_DIR) / name
        try:
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(f.content)
        except OSError as exc:
            _notify(event, "Upload fehlgeschlagen", str(exc))
            return
        if simple:
            notes = []
            problem = mesh_utils.make_mujoco_ready(dest, notes)
            if not problem:
                try:
                    editor.controller.create_mesh(editor.get_selected_parent(), dest.resolve())
                except Exception as exc:  # pragma: no cover - Laufzeit
                    _notify(event, "Import fehlgeschlagen", str(exc))
                    return
                _notify(event, "Mesh eingefuegt",
                        f"{dest.name} nach meshes/ gespeichert und in die Szene gelegt. "
                        + " ".join(notes))
                return
            # z.B. > 200000 Dreiecke: in Teile zerlegen statt ablehnen
            print(f"[run_editor] {problem} -> wird ueber den CAD-Import zerlegt.")
        _import_and_insert(editor, dest, event)


def _guard_create_mesh(editor) -> None:
    """Kein Mesh in die Szene lassen, das MuJoCo nicht laden kann.

    Alle Wege, ein Mesh einzufuegen (Upload-Knopf, "Add Assets from File",
    Objaverse), laufen durch controller.create_mesh(). Das legt das Blueprint
    erst in den Szenen-Zustand und rendert es dann - fliegt dabei ein Fehler
    ("stl_decoder: number of faces should be between 1 and 200000"), steckt
    das kaputte Objekt schon in der Szene und auch das Speichern geht nicht
    mehr. Darum wird hier VORHER geprueft: ASCII-STL wird repariert, zu grosse
    Netze und fremde Formate (PLY/GLB aus dem Ordner-Scan) laufen durch den
    CAD-Import und kommen als Gruppe aus Einzelteilen in die Szene.
    """
    ctrl = editor.controller
    orig = ctrl.create_mesh

    def create_mesh(parent_name, mesh_path, *args, **kwargs):
        path = Path(mesh_path)
        notes = []
        try:
            problem = mesh_utils.make_mujoco_ready(path, notes)
        except Exception as exc:  # pragma: no cover - Laufzeit
            problem = f"{path.name}: Pruefung fehlgeschlagen ({exc})"
        for note in notes:
            print(f"[run_editor] {note}")
        if not problem:
            return orig(parent_name, mesh_path, *args, **kwargs)
        if cad_import.is_importable(path) and cad_import.trimesh_available():
            print(f"[run_editor] {problem} -> wird ueber den CAD-Import zerlegt.")
            _import_and_insert(editor, path)
            return None
        msg = (f"{problem} Das Objekt wurde NICHT eingefuegt (die Szene bliebe "
               "sonst kaputt).")
        print(f"[run_editor] Mesh abgelehnt: {problem}", file=sys.stderr)
        _broadcast(editor, "Mesh nicht verwendbar", msg)
        raise RuntimeError(msg)

    ctrl.create_mesh = create_mesh


def _cad_candidates() -> list:
    """Dateien in meshes/ (+ uploads/), die erst importiert werden muessen."""
    out = []
    for folder in (MESHES_DIR, UPLOADS_DIR):
        if not folder.is_dir():
            continue
        for p in sorted(folder.iterdir()):
            if p.is_file() and cad_import.is_importable(p)                     and p.suffix.lower() not in mesh_utils.MJ_MESH_SUFFIXES:
                out.append(p)
    return out


def _install_cad_controls(editor) -> None:
    """Ordner "CAD-Import": Einstellungen + Dateien aus meshes/ einfuegen."""
    server = editor.layout.server
    cad_ok = cad_import.occ_available()
    try:
        with server.gui.add_folder("CAD-Import", order=1.35, expand_by_default=False):
            if not cad_ok:
                server.gui.add_markdown(
                    "STEP/IGES inaktiv - OpenCascade fehlt: "
                    "`.venv/bin/pip install cadquery-ocp`")
            quality = server.gui.add_dropdown(
                "Genauigkeit", options=tuple(cad_import.QUALITY),
                initial_value=str(_CAD_OPTS["quality"]),
                hint="Feinheit der Rundungen (fine = mehr Dreiecke, langsamer).")
            scale = server.gui.add_number(
                "Skalierung", initial_value=float(_CAD_OPTS["scale"]),
                min=0.0, max=1000.0, step=0.0001,
                hint="0 = automatisch (CAD: Einheit aus der Datei -> Meter, "
                     "Meshes unveraendert). Sonst fester Faktor, z.B. 0.001.")
            place = server.gui.add_dropdown(
                "Platzierung", options=cad_import.PLACEMENTS,
                initial_value=str(_CAD_OPTS["place"]),
                hint="floor = auf den Boden stellen, center = zusaetzlich mittig "
                     "um den Ursprung, cad = CAD-Koordinaten behalten.")
            small = server.gui.add_number(
                "Nur Optik unter (m)", initial_value=float(_CAD_OPTS["min_collision_size"]),
                min=0.0, max=10.0, step=0.005,
                hint="Teile mit kleinerer Diagonale (Schrauben, Knoepfe) nehmen "
                     "nicht an der Kollision teil.")
            files = server.gui.add_dropdown(
                "Datei in meshes/", options=("(keine)",), initial_value="(keine)",
                hint="CAD-/Mesh-Dateien, die nach meshes/ bzw. meshes/uploads/ "
                     "kopiert wurden.")
            btn_refresh = server.gui.add_button("Liste aktualisieren")
            btn_import = server.gui.add_button("Importieren & einfuegen", color="green")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] CAD-Controls nicht verfuegbar: {exc}", file=sys.stderr)
        return

    def refresh() -> None:
        names = tuple(p.name for p in _cad_candidates()) or ("(keine)",)
        files.options = names
        if files.value not in names:
            files.value = names[0]

    refresh()

    @quality.on_update
    def _q(_evt) -> None:
        _CAD_OPTS["quality"] = str(quality.value)

    @scale.on_update
    def _s(_evt) -> None:
        _CAD_OPTS["scale"] = float(scale.value or 0.0)

    @place.on_update
    def _p(_evt) -> None:
        _CAD_OPTS["place"] = str(place.value)

    @small.on_update
    def _m(_evt) -> None:
        _CAD_OPTS["min_collision_size"] = float(small.value or 0.0)

    @btn_refresh.on_click
    def _r(_evt) -> None:
        refresh()

    @btn_import.on_click
    def _i(event) -> None:
        match = [p for p in _cad_candidates() if p.name == files.value]
        if not match:
            _notify(event, "Keine Datei gewaehlt",
                    "CAD-Datei nach scene_editor/meshes/ kopieren, 'Liste "
                    "aktualisieren', dann waehlen - oder oben 'Eigene Datei "
                    "hochladen' benutzen.")
            return
        btn_import.disabled = True
        try:
            _import_and_insert(editor, match[0], event)
        finally:
            btn_import.disabled = False


def _install_mesh_scale_control(editor) -> None:
    """Skalier-Control fuer Meshes (fehlt im eingebauten Editor).

    Der Properties-Panel des Editors kann nur Box/Zylinder/Kugel-Masse aendern,
    aber importierte Meshes/STLs nicht skalieren. Hier: gewaehltes Mesh oben
    unter "Elements" waehlen, Faktor eingeben, anwenden. Loest auch mm->m
    (CAD-STL in mm -> Faktor 0.001).
    """
    from robits.sim.blueprints import MeshBlueprint

    server = editor.layout.server
    ctrl = editor.controller
    try:
        with server.gui.add_folder("Mesh skalieren", order=1.4,
                                   expand_by_default=True):
            num = server.gui.add_number(
                "Faktor", initial_value=1.0, min=0.0001, max=10000.0, step=0.01,
                hint="Skaliert das oben gewaehlte Mesh. CAD-STL in mm -> 0.001.")
            btn = server.gui.add_button("Auf gewaehltes Mesh anwenden")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Skalier-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    # Beim Auswaehlen eines Meshes den aktuellen Faktor ins Feld holen (viser
    # haengt zusaetzliche on_update-Callbacks an, ersetzt die vorhandenen nicht).
    @editor.layout.element_list.on_update
    def _sync_scale(_evt) -> None:
        bp = ctrl.state.blueprints.get(editor.layout.element_list.value)
        if isinstance(bp, MeshBlueprint):
            try:
                num.value = float(getattr(bp, "scale", 1.0) or 1.0)
            except Exception:
                pass

    @btn.on_click
    def _apply_scale(event) -> None:
        name = editor.layout.element_list.value
        bp = ctrl.state.blueprints.get(name)
        if not isinstance(bp, MeshBlueprint):
            _notify(event, "Kein Mesh gewaehlt",
                    "Bitte oben unter 'Elements' ein importiertes Mesh auswaehlen.")
            return
        try:
            factor = float(num.value)
        except (TypeError, ValueError):
            _notify(event, "Ungueltiger Faktor", "Bitte eine Zahl eingeben.")
            return
        if factor <= 0:
            _notify(event, "Ungueltiger Faktor", "Faktor muss > 0 sein.")
            return
        try:
            ctrl.state.update(name, scale=factor)
            ctrl.renderer.render_from_state(list(ctrl.state.blueprints.values()))
            editor.layout.element_list.value = name  # Auswahl/Gizmo wiederherstellen
            ctrl.select(name)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Skalieren fehlgeschlagen: {exc}", file=sys.stderr)
            _notify(event, "Skalieren fehlgeschlagen", f"{type(exc).__name__}: {exc}")
            return
        _notify(event, "Mesh skaliert", f"Faktor {factor} angewendet.")


def _check_meshes_at_startup() -> None:
    """STLs in meshes/ vorab pruefen: ASCII reparieren, zu grosse melden.

    Der Ordner-Scan ("Add Assets from File") bietet alles an, was in meshes/
    liegt. ASCII-STL wird hier gleich binaer neu geschrieben; zu grosse Netze
    zerlegt der Tuersteher beim Einfuegen automatisch (siehe _guard_create_mesh).
    """
    for stl in sorted(MESHES_DIR.glob("*.stl")):
        notes = []
        try:
            problem = mesh_utils.make_mujoco_ready(stl, notes)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] {stl.name}: Pruefung fehlgeschlagen ({exc})",
                  file=sys.stderr)
            continue
        for note in notes:
            print(f"[run_editor] {note}")
        if problem:
            print(f"[run_editor] Hinweis: {problem} Beim Einfuegen wird es "
                  "automatisch in Teile zerlegt.")


def _install_patches() -> None:
    """Anpassungen am Editor/robits fuer grosse CAD-Importe (siehe oben).

    Schlaegt eine davon fehl (andere Paketversion), laeuft der Editor trotzdem -
    nur eben ohne diese Verbesserung.
    """
    for patch in (_patch_renderer_for_many_objects, _patch_mesh_rendering,
                  _patch_mesh_export, _hide_cad_parts_in_asset_scan):
        try:
            patch()
        except Exception as exc:  # pragma: no cover - andere Paketversion
            print(f"[run_editor] WARNUNG: {patch.__name__} nicht moeglich: {exc}",
                  file=sys.stderr)


_install_patches()


_orig_get_scene_editor = _editor_cli.get_scene_editor


def _get_scene_editor_with_extras(blueprints=None):
    editor = _orig_get_scene_editor(blueprints)
    _guard_create_mesh(editor)
    _install_save_control(editor)
    _install_rename_control(editor)
    _install_upload_button(editor)
    _install_mesh_scale_control(editor)
    _install_cad_controls(editor)
    print(f"[run_editor] Editor laeuft: http://127.0.0.1:{EDITOR_PORT}")
    return editor


_editor_cli.get_scene_editor = _get_scene_editor_with_extras


# ---------------------------------------------------------------------------
# 'prompt': Umgebung per Text erzeugen UND danach im Editor oeffnen.
# Die eingebaute CLI schreibt nur eine Datei und gibt einen Hinweis aus - wer
# ueber launch.sh/GUI startet, wartet dann vergeblich auf den Editor.
# ---------------------------------------------------------------------------
def _run_prompt(rest) -> int:
    text = " ".join(a for a in rest if not a.startswith("-")).strip()
    if not text:
        print("[run_editor] Bitte eine Beschreibung angeben, z.B.:\n"
              '    ./launch.sh prompt "a kitchen with a table and two boxes"',
              file=sys.stderr)
        return 2
    if not os.environ.get("OPENAI_API_KEY"):
        print("[run_editor] Fuer 'prompt' wird ein OpenAI-API-Key gebraucht:\n"
              "    export OPENAI_API_KEY=sk-...\n"
              "Ohne Key: leere Umgebung starten und die Objekte selbst setzen.",
              file=sys.stderr)
        return 2

    base = sanitize_env_name(text)[:40] or "prompt"
    name, i = base, 2
    while env_store.env_dir(name).exists() or (SCENES_DIR / f"{name}.xml").exists():
        name = f"{base}_{i}"
        i += 1

    print(f"[run_editor] Erzeuge Umgebung aus Text -> scenes/{name}/ (kann dauern) ...")
    with tempfile.TemporaryDirectory(prefix="scene_prompt_") as td:
        raw = Path(td) / "prompt.xml"
        try:
            _editor_cli.cli.main(["prompt", "--output-model-name", str(raw), text],
                                 standalone_mode=False)
        except Exception as exc:
            print(f"[run_editor] Text-Generierung fehlgeschlagen: {exc}", file=sys.stderr)
            return 1
        if not raw.is_file():
            print("[run_editor] Es wurde keine Umgebung erzeugt (Antwort nicht verwertbar).",
                  file=sys.stderr)
            return 1
        # Meshes der generierten Szene (z.B. Objaverse-Downloads) mit in den Ordner.
        out = env_store.make_self_contained(raw, env_store.env_dir(name))

    print(f"[run_editor] Umgebung erzeugt: scenes/{name}/ - oeffne sie im Editor.")
    global INITIAL_NAME
    INITIAL_NAME = name
    try:
        _editor_cli.cli.main(["edit", str(out)], standalone_mode=False)
    except Exception as exc:
        print(f"[run_editor] Editor konnte die erzeugte Umgebung nicht oeffnen: {exc}",
              file=sys.stderr)
        return 1
    return 0


def main() -> int:
    _check_meshes_at_startup()
    argv = sys.argv[1:]
    if argv and argv[0] == "prompt":
        return _run_prompt(argv[1:])
    if argv and argv[0] == "edit":
        target = Path(argv[1]) if len(argv) > 1 else None
        if target is not None and not target.is_file():
            found = bes.resolve_env_path(str(target))
            if found.is_file():
                sys.argv[2] = str(found)
            else:
                envs = ", ".join(bes.available_envs()) or "(keine)"
                print(f"[run_editor] Umgebung nicht gefunden: {target}\n"
                      f"             Vorhanden in scenes/: {envs}", file=sys.stderr)
                return 2
    _editor_cli.cli()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
