#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  g1_gui.py — Grafisches Start-Menue fuer den G1-Stack (Sim / Real / Szenen).
#
#  Ziel: Im normalen Betrieb muss man KEIN cmd-Fenster mehr anfassen. Dieses
#  Tkinter-Fenster ersetzt die Text-Menues aus:
#     - g1pilot/start.sh                 (Sim / Real starten)
#     - unitree_mujoco/scene_editor/launch.sh   (Umgebungen bearbeiten)
#
#  Es implementiert die Orchestrierung NICHT neu, sondern ruft die bewaehrten
#  Skripte env-getrieben auf:
#     Sim / Real  ->  start.sh --yes           (alle Optionen als Env-Vars)
#     Szenen      ->  scene_editor/launch.sh <cmd> [datei]
#  Die Ausgabe der Skripte (Docker/Compose/Viewer) landet live in einer
#  In-App-Log-View mit Stop-Button — kein Terminal noetig.
#
#  EIN Fenster: Menue, Options-Views und Log-Views werden im selben Fenster
#  ausgetauscht (Router). Laufende Prozesse bleiben in der Registry — man geht
#  per '‹ Menue' zurueck (Prozess laeuft weiter) und ueber 'Laufende Prozesse'
#  wieder hin. Schliessen = ein einziges Fenster schliessen.
#
#  Start:  python3 g1_gui.py        (start.sh startet das automatisch)
#
#  Faellt Tkinter/DISPLAY aus, benutzt start.sh weiter das Text-Menue.
# ════════════════════════════════════════════════════════════════════════
from __future__ import annotations

import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, simpledialog, filedialog
except Exception as exc:  # pragma: no cover - nur wenn tkinter fehlt
    sys.stderr.write(
        "[g1_gui] Tkinter ist nicht verfuegbar (%s).\n"
        "         Bitte das klassische Text-Menue nutzen: ./start.sh --menu\n" % exc
    )
    sys.exit(2)


# ── Pfade (alles relativ zu diesem Skript, damit der Aufrufort egal ist) ──
HERE = Path(__file__).resolve().parent                       # .../g1pilot
REPO_ROOT = HERE.parent                                      # Repo-Wurzel
START_SH = HERE / "start.sh"
SCENE_DIR = REPO_ROOT / "unitree_mujoco" / "scene_editor"
SCENES_DIR = SCENE_DIR / "scenes"
LAUNCH_SH = SCENE_DIR / "launch.sh"
SETUP_SH = SCENE_DIR / "setup.sh"
SCENE_VENV = SCENE_DIR / ".venv" / "bin" / "python"
BUILD_ENV_PY = SCENE_DIR / "build_env_scene.py"
EXPORT_DIR = SCENE_DIR / "export"          # Default-Ziel fuer Umgebungs-Zips

# Umgebungen sind Ordner scenes/<name>/ (umgebung.xml + meshes/). Das Format
# kennt genau EIN Modul (env_store.py, nur Standardbibliothek) - Editor,
# Generator, launch.sh, start.sh und diese GUI benutzen es gemeinsam.
sys.path.insert(0, str(SCENE_DIR))
import env_store  # noqa: E402

# Port des Editor-Webservers (viser). Muss zu run_editor.py/launch.sh passen.
EDITOR_PORT = os.environ.get("SCENE_EDITOR_PORT", "8080")
EDITOR_URL = f"http://127.0.0.1:{EDITOR_PORT}"

# Label der "keine eigene Umgebung"-Auswahl (an mehreren Stellen gebraucht).
DEFAULT_ENV_LABEL = "Standard — aktuelles Terrain (scene.xml)"

# Dokumente, die aus dem Menue heraus geoeffnet werden koennen. Vollstaendige
# Uebersicht: docs/README.md. Jedes Thema hat eine Anleitung (Anwender) und
# ein Technik-Dokument (Entwickler) -- siehe g1pilot/docs/.
DOCS_DIR = HERE / "docs"
DOCS = [
    ("Dokumentation — Uebersicht", DOCS_DIR / "README.md"),
    ("README (G1Pilot)", HERE / "README.md"),
    ("Installation & Ersteinrichtung", DOCS_DIR / "01_installation.md"),
    ("Architektur (Entwickler)", DOCS_DIR / "02_architektur.md"),
    ("Arm-Manipulation — Anleitung", DOCS_DIR / "10_arm_manipulation_anleitung.md"),
    ("Arm-Manipulation — Technik", DOCS_DIR / "11_arm_manipulation_technik.md"),
    ("Arm-API — Anleitung", DOCS_DIR / "20_arm_api_anleitung.md"),
    ("Arm-API — Technik", DOCS_DIR / "21_arm_api_technik.md"),
    ("Locomotion — Anleitung", DOCS_DIR / "30_loco_anleitung.md"),
    ("Locomotion — Technik", DOCS_DIR / "31_loco_technik.md"),
    ("Teleoperation — Anleitung", DOCS_DIR / "40_teleoperation_anleitung.md"),
    ("Teleoperation — Technik", DOCS_DIR / "41_teleoperation_technik.md"),
    ("Navigation — Anleitung", DOCS_DIR / "50_navigation_anleitung.md"),
    ("Navigation — Technik", DOCS_DIR / "51_navigation_technik.md"),
    ("Inspire-FTP-Haende — Anleitung", DOCS_DIR / "60_inspire_haende_anleitung.md"),
    ("Inspire-FTP-Haende — Technik", DOCS_DIR / "61_inspire_haende_technik.md"),
    ("Echter Roboter — Sicherheit & Ablauf", DOCS_DIR / "70_echtroboter_anleitung.md"),
]

# ── Farb-/Stilkonstanten ────────────────────────────────────────────────
BG = "#1f232b"
CARD = "#2a2f3a"
FG = "#e8eaed"
MUTED = "#9aa0aa"
ACCENT = "#4c8bf5"
GREEN = "#3ecf7a"
RED = "#e5484d"
AMBER = "#e0a13b"


# ════════════════════════════════════════════════════════════════════════
#  Hilfsfunktionen (Host-Umgebung abfragen)
# ════════════════════════════════════════════════════════════════════════
def have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


def docker_ready() -> bool:
    """True, wenn 'docker' existiert und der Daemon erreichbar ist."""
    if not have("docker"):
        return False
    try:
        r = subprocess.run(
            ["docker", "info"], capture_output=True, timeout=6
        )
        return r.returncode == 0
    except Exception:
        return False


def list_scenes() -> list:
    """Alle brauchbaren Umgebungen (env_store.Env: .name, .xml, .folder)."""
    return env_store.list_envs()


def list_broken_scenes() -> list[tuple[Path, str]]:
    """Eintraege in scenes/, die nicht als Umgebung taugen (+ Grund)."""
    return env_store.scan()[1]


def scene_python() -> str:
    """Python fuer die Szenen-Skripte: bevorzugt das Editor-venv (hat mujoco und
    kann die kombinierte Szene damit vorab pruefen), sonst der System-Python."""
    if SCENE_VENV.exists():
        return str(SCENE_VENV)
    return sys.executable or "python3"


def build_env_scene(env_name: str, inspire: bool) -> tuple[bool, str]:
    """Kombinierte Szene (G1 + Umgebung) erzeugen. -> (ok, Ausgabe)

    Genau das macht start.sh vor dem Container-Start auch; wir ziehen es in die
    GUI vor, damit ein Fehler in der Umgebung SOFORT sichtbar wird statt erst
    als stiller Rueckfall auf das Standard-Terrain.
    """
    if not BUILD_ENV_PY.is_file():
        return False, f"{BUILD_ENV_PY} fehlt."
    try:
        r = subprocess.run(
            [scene_python(), str(BUILD_ENV_PY), "--env", env_name,
             "--inspire", "1" if inspire else "0"],
            capture_output=True, text=True, timeout=180, cwd=str(SCENE_DIR),
        )
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"
    out = (r.stdout or "") + (r.stderr or "")
    return r.returncode == 0, out.strip()


def scene_editor_ready() -> bool:
    """True, wenn das scene_editor-virtualenv eingerichtet ist."""
    return SCENE_VENV.exists()


def detect_nics() -> list[tuple[str, str]]:
    """Physische Netzwerk-Interfaces + IPv4 ermitteln (wie start.sh, in Python).

    Rueckgabe: Liste (name, ip). Bei fehlendem 'ip'-Tool leere Liste.
    """
    nics: list[tuple[str, str]] = []
    if not have("ip"):
        return nics
    try:
        out = subprocess.run(
            ["ip", "-o", "link", "show"], capture_output=True, text=True, timeout=5
        ).stdout
    except Exception:
        return nics
    for line in out.splitlines():
        # Format: "2: eth0: <BROADCAST,...> ..."
        parts = line.split(": ")
        if len(parts) < 2:
            continue
        name = parts[1].split("@")[0].strip()
        if not name or name == "lo":
            continue
        if name.startswith(("docker", "veth", "br-", "virbr")):
            continue
        ip = ""
        try:
            addr = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", name],
                capture_output=True, text=True, timeout=5,
            ).stdout
            for al in addr.splitlines():
                toks = al.split()
                if "inet" in toks:
                    ip = toks[toks.index("inet") + 1]
                    break
        except Exception:
            pass
        nics.append((name, ip))
    return nics


def open_path(path: Path) -> None:
    """Datei mit dem System-Standard oeffnen (Doku etc.)."""
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(str(path))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)])
    except Exception as exc:
        messagebox.showerror("Oeffnen fehlgeschlagen", f"{path}\n\n{exc}")


def open_url(url: str) -> bool:
    """Eine URL im System-Browser oeffnen. True bei Erfolg.

    Der Scene-Editor oeffnet den Browser nur bei einem TTY selbst — unter der
    GUI laeuft er als Subprozess (Pipe, kein TTY), daher uebernimmt das die GUI.
    Deckt Linux/Mac/Windows und WSL2 ab.
    """
    try:
        if webbrowser.open(url):
            return True
    except Exception:
        pass
    # WSL2/Windows/Linux-Fallbacks (analog start.sh)
    for exe in ("wslview", "xdg-open", "sensible-browser", "x-www-browser",
                "google-chrome", "chromium", "firefox"):
        if have(exe):
            try:
                subprocess.Popen([exe, url],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except Exception:
                pass
    if os.name == "nt":
        try:
            os.startfile(url)  # type: ignore[attr-defined]
            return True
        except Exception:
            pass
    return False


# ════════════════════════════════════════════════════════════════════════
#  Wiederverwendbare Widgets
# ════════════════════════════════════════════════════════════════════════
def section(parent, text: str) -> tk.Frame:
    """Ein abgesetzter 'Card'-Rahmen mit Ueberschrift."""
    outer = tk.Frame(parent, bg=CARD, bd=0, highlightthickness=0)
    outer.pack(fill="x", padx=14, pady=6)
    tk.Label(outer, text=text, bg=CARD, fg=ACCENT,
             font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
    inner = tk.Frame(outer, bg=CARD)
    inner.pack(fill="x", padx=12, pady=(0, 10))
    return inner


def toggle_row(parent, label: str, var: tk.BooleanVar, hint: str = "") -> tk.Frame:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    cb = tk.Checkbutton(row, text=label, variable=var, bg=CARD, fg=FG,
                        selectcolor=BG, activebackground=CARD, activeforeground=FG,
                        anchor="w", font=("TkDefaultFont", 10))
    cb.pack(side="left")
    if hint:
        tk.Label(row, text=hint, bg=CARD, fg=MUTED,
                 font=("TkDefaultFont", 9)).pack(side="left", padx=8)
    return row


def field_row(parent, label: str, var: tk.StringVar, width: int = 18) -> tk.Entry:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    tk.Label(row, text=label, bg=CARD, fg=FG, width=22, anchor="w",
             font=("TkDefaultFont", 10)).pack(side="left")
    ent = tk.Entry(row, textvariable=var, width=width, bg=BG, fg=FG,
                   insertbackground=FG, relief="flat")
    ent.pack(side="left", padx=4, ipady=3)
    return ent


def primary_button(parent, text: str, cmd, color: str = ACCENT) -> tk.Button:
    return tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                     activebackground=color, relief="flat",
                     font=("TkDefaultFont", 11, "bold"), padx=18, pady=8)


def big_button(parent, icon, title, subtitle, color, cmd):
    """Grosse anklickbare Karte fuers Hauptmenue."""
    card = tk.Frame(parent, bg=CARD, cursor="hand2")
    card.pack(fill="x", pady=7)
    bar = tk.Frame(card, bg=color, width=6)
    bar.pack(side="left", fill="y")
    inner = tk.Frame(card, bg=CARD)
    inner.pack(side="left", fill="both", expand=True, padx=14, pady=12)
    tk.Label(inner, text=f"{icon}  {title}", bg=CARD, fg=FG,
             font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
    tk.Label(inner, text=subtitle, bg=CARD, fg=MUTED,
             font=("TkDefaultFont", 10)).pack(anchor="w")
    for w in (card, inner, bar, *inner.winfo_children()):
        w.bind("<Button-1>", lambda _e, c=cmd: c())
    return card


def nav_header(frame, app, title, subtitle, color=FG):
    """Kopfzeile einer Unteransicht mit '‹ Menue'-Zurueck-Button."""
    head = tk.Frame(frame, bg=BG)
    head.pack(fill="x", padx=16, pady=(12, 2))
    top = tk.Frame(head, bg=BG)
    top.pack(fill="x")
    tk.Button(top, text="‹  Menue", command=app.show_menu, bg=CARD, fg=FG,
              relief="flat", padx=10, pady=4).pack(side="left")
    tk.Label(top, text=title, bg=BG, fg=color,
             font=("TkDefaultFont", 15, "bold")).pack(side="left", padx=10)
    tk.Label(head, text=subtitle, bg=BG, fg=MUTED).pack(anchor="w", pady=(2, 6))


class ScrollableFrame(tk.Frame):
    """Ein vertikal scrollbarer Bereich. Inhalt kommt in '.body'."""

    def __init__(self, parent):
        super().__init__(parent, bg=BG)
        self._canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)
        self.body = tk.Frame(self._canvas, bg=BG)
        self._win = self._canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind(
            "<Configure>",
            lambda _e: self._canvas.configure(scrollregion=self._canvas.bbox("all")))
        self._canvas.bind(
            "<Configure>",
            lambda e: self._canvas.itemconfigure(self._win, width=e.width))
        # Mausrad nur aktiv, solange der Zeiger ueber diesem Bereich ist.
        self.bind("<Enter>", self._bind_wheel)
        self.bind("<Leave>", self._unbind_wheel)
        self.bind("<Destroy>", self._unbind_wheel)

    def _bind_wheel(self, _e=None):
        self._canvas.bind_all("<MouseWheel>", self._on_wheel)
        self._canvas.bind_all("<Button-4>", self._on_wheel)
        self._canvas.bind_all("<Button-5>", self._on_wheel)

    def _unbind_wheel(self, _e=None):
        try:
            self._canvas.unbind_all("<MouseWheel>")
            self._canvas.unbind_all("<Button-4>")
            self._canvas.unbind_all("<Button-5>")
        except tk.TclError:
            pass

    def _on_wheel(self, e):
        if e.num == 4 or getattr(e, "delta", 0) > 0:
            self._canvas.yview_scroll(-1, "units")
        elif e.num == 5 or getattr(e, "delta", 0) < 0:
            self._canvas.yview_scroll(1, "units")


# Queue-Sentinels (Kontrollmarker, unterscheidbar von Log-Zeilen/Strings):
_EOF = object()   # Reader: Prozess-Ausgabe zu Ende (Prozess beendet)
_DONE = object()  # Stop-Worker: Aufraeumen (docker down) abgeschlossen -> finalisieren
# viser-Banner des Editors ("HTTP │ http://127.0.0.1:8081"). Ist 8080 belegt
# (z.B. alter Editor), weicht viser auf 8081, 8082 ... aus -- die echte URL
# steht nur in dieser Ausgabezeile.
_EDITOR_URL_RE = re.compile(r"http://(?:127\.0\.0\.1|localhost):\d+")


# ════════════════════════════════════════════════════════════════════════
#  Konsole-View: startet einen Subprozess und zeigt dessen Ausgabe live
# ════════════════════════════════════════════════════════════════════════
class ConsoleFrame(tk.Frame):
    """View, die einen Subprozess ausfuehrt und dessen Ausgabe streamt.

    - Bleibt in der App-Registry, auch wenn man zurueck ins Menue navigiert:
      der Prozess laeuft weiter, das Log wird weiter mitgeschrieben, und man
      kann jederzeit ueber 'Laufende Prozesse' im Menue zurueckkehren.
    - 'Stoppen' beendet den Prozess (SIGTERM) + optionales Aufraeum-Kommando.
    - Thread-sicher via Queue + after()-Polling (Tkinter ist nicht threadsafe).
    """

    ephemeral = False  # nicht zerstoeren, wenn eine andere View gezeigt wird

    def __init__(self, parent, app, title: str, argv: list[str], *,
                 cwd: Path, env: dict | None = None,
                 stop_cmd: list[str] | None = None,
                 stop_note: str = "",
                 browser_url: str | None = None):
        super().__init__(parent, bg=BG)
        self.app = app
        self.view_title = title
        self._title = title
        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._stop_cmd = stop_cmd
        self._stop_note = stop_note
        self._browser_url = browser_url
        self._server_url: str | None = None  # echte Editor-URL aus der Ausgabe
        self._proc: subprocess.Popen | None = None
        self._queue: queue.Queue = queue.Queue()  # Log-Strings + _EOF/_DONE-Marker
        self._stopping = False
        self._finished = False
        self._alive = True
        self._after_id = None

        # Kopfzeile: Zurueck + Titel + Status.
        head = tk.Frame(self, bg=CARD)
        head.pack(fill="x")
        tk.Button(head, text="‹  Menue", command=app.show_menu, bg=CARD, fg=FG,
                  relief="flat", padx=10, pady=6).pack(side="left", padx=6, pady=6)
        tk.Label(head, text=title, bg=CARD, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(side="left", padx=6)
        self._status = tk.Label(head, text="laeuft…", bg=CARD, fg=AMBER,
                                font=("TkDefaultFont", 10, "bold"))
        self._status.pack(side="right", padx=12)

        # Log-Textfeld.
        wrap = tk.Frame(self, bg=BG)
        wrap.pack(fill="both", expand=True, padx=10, pady=(8, 4))
        self._text = tk.Text(wrap, bg="#12151b", fg="#d7dbe0", insertbackground=FG,
                             wrap="none", relief="flat", font=("TkFixedFont", 9),
                             state="disabled")
        yscroll = ttk.Scrollbar(wrap, orient="vertical", command=self._text.yview)
        xscroll = ttk.Scrollbar(self, orient="horizontal", command=self._text.xview)
        self._text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self._text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)
        xscroll.pack(fill="x", padx=10)

        # Fussleiste.
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", padx=10, pady=8)
        self._stop_btn = tk.Button(foot, text="■  Stoppen", command=self.stop,
                                   bg=RED, fg="white", activebackground="#c93b3f",
                                   relief="flat", font=("TkDefaultFont", 10, "bold"),
                                   padx=14, pady=6)
        self._stop_btn.pack(side="left")
        tk.Label(foot, text="'Menue' laesst den Prozess im Hintergrund weiterlaufen.",
                 bg=BG, fg=MUTED, font=("TkDefaultFont", 9)).pack(side="left", padx=12)

        self._append(f"$ {' '.join(argv)}\n\n")
        self._start()
        self._after_id = self.after(80, self._drain)
        if self._browser_url and self._proc is not None:
            threading.Thread(target=self._browser_waiter, daemon=True).start()

    # ── Status fuer die Menue-Liste ───────────────────────────────────────
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def state(self) -> str:
        if self.is_running():
            return "stoppe…" if self._stopping else "laeuft"
        return "gestoppt" if self._stopping else "beendet"

    # ── intern ────────────────────────────────────────────────────────────
    def _start(self) -> None:
        try:
            self._proc = subprocess.Popen(
                self._argv, cwd=str(self._cwd), env=self._env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, text=True,
            )
        except Exception as exc:
            self._append(f"[Fehler beim Start] {exc}\n")
            self._finalize(-1)
            return
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self._proc and self._proc.stdout
        for line in self._proc.stdout:
            if self._browser_url and self._server_url is None:
                m = _EDITOR_URL_RE.search(line)
                if m:
                    self._server_url = m.group(0)
            self._queue.put(line)
        self._proc.wait()
        self._queue.put(_EOF)  # Prozess-Ausgabe zu Ende

    def _browser_waiter(self) -> None:
        """Wartet, bis der Editor-Webserver lauscht, und oeffnet dann den Browser.

        Bewusst eine echte HTTP-Anfrage (wie ein Browser) statt eines rohen
        TCP-Connects: der Editor-Server (viser) bedient HTTP UND WebSocket auf
        demselben Port; ein sofort geschlossener TCP-Connect saehe fuer ihn wie
        ein abgebrochener WebSocket-Handshake aus und wuerde einen (harmlosen)
        Fehler-Traceback loggen.

        Die URL kommt aus der Editor-Ausgabe (_reader), NICHT fest 8080: sonst
        oeffnet der Browser einen alten, evtl. haengenden Editor auf 8080,
        waehrend der neue auf 8081 laeuft.
        """
        deadline = time.monotonic() + 180  # max ~3 min (deckt ersten Start ab)
        while self._alive and not self._stopping and time.monotonic() < deadline:
            url = self._server_url
            if url and self._server_responds(url):
                if self._alive:
                    self._queue.put(f"\n[GUI] Oeffne Editor im Browser: {url}\n")
                    open_url(url)
                return
            if self._proc is not None and self._proc.poll() is not None:
                return  # Prozess beendet, ohne je zu lauschen
            time.sleep(0.5)

    @staticmethod
    def _server_responds(url: str) -> bool:
        """True, sobald der HTTP-Server irgendeine Antwort liefert."""
        try:
            urllib.request.urlopen(url, timeout=1).close()
            return True
        except urllib.error.HTTPError:
            return True  # Server antwortet (nur mit Fehlerstatus) -> laeuft
        except (urllib.error.URLError, OSError):
            return False  # noch nicht erreichbar

    def _drain(self) -> None:
        if not self._alive:
            return
        try:
            while True:
                item = self._queue.get_nowait()
                if item is _EOF:
                    # Prozess-Ausgabe endet. Bei benutzerinitiiertem Stop NICHT
                    # sofort finalisieren — erst wartet noch das Aufraeum-Kommando
                    # (docker down); der Stop-Worker meldet danach _DONE. So gilt
                    # "beendet" wirklich erst, wenn MuJoCo/Container zu sind.
                    if not self._stopping:
                        self._finalize()
                        return
                elif item is _DONE:
                    self._finalize()
                    return
                else:
                    self._append(item)
        except queue.Empty:
            pass
        except tk.TclError:
            return  # View wurde zwischenzeitlich zerstoert
        self._after_id = self.after(80, self._drain)

    def _append(self, text: str) -> None:
        self._text.configure(state="normal")
        self._text.insert("end", text)
        # Speicher begrenzen: hoechstens ~4000 Zeilen behalten.
        last = int(self._text.index("end-1c").split(".")[0])
        if last > 4000:
            self._text.delete("1.0", f"{last - 4000}.0")
        self._text.see("end")
        self._text.configure(state="disabled")

    def _finalize(self, rc: int | None = None) -> None:
        self._finished = True
        if rc is None:
            rc = self._proc.returncode if self._proc else -1
        user_stopped = self._stopping
        if user_stopped:
            self._status.configure(text="gestoppt", fg=MUTED)
        elif rc == 0:
            self._status.configure(text="beendet (ok)", fg=GREEN)
        else:
            self._status.configure(text=f"beendet (Code {rc})", fg=RED)
        self._stop_btn.configure(state="disabled")
        # Sauber (ab)geschlossen -> App entscheidet ueber Auto-Rueckkehr/Refresh.
        self.app.notify_finished(self, user_stopped=user_stopped)

    def terminate_editor(self) -> None:
        """Beim Schliessen des Launchers: Web-Editor mitbeenden.

        Sonst laeuft er verwaist weiter (Ausgabe-Pipe ohne Leser -> haengt),
        blockiert Port 8080 und der naechste Editor weicht auf 8081 aus.
        Docker-Stacks sind davon nicht betroffen (die laufen bewusst weiter).
        """
        if self._browser_url and self.is_running():
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass

    def stop(self) -> None:
        if self._stopping or not self.is_running():
            return
        self._stopping = True
        self._status.configure(text="stoppe…", fg=AMBER)
        self._stop_btn.configure(state="disabled")
        self._append("\n[Stoppen angefordert]\n")

        def worker():
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=12)
                except Exception:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
            if self._stop_cmd:
                self._queue.put(f"\n$ {' '.join(self._stop_cmd)}\n")
                try:
                    r = subprocess.run(self._stop_cmd, cwd=str(self._cwd),
                                       capture_output=True, text=True, timeout=90)
                    self._queue.put(r.stdout + r.stderr)
                except Exception as exc:
                    self._queue.put(f"[down-Fehler] {exc}\n")
            self._queue.put(_DONE)

        threading.Thread(target=worker, daemon=True).start()

    def destroy(self) -> None:
        self._alive = False
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        super().destroy()


# ════════════════════════════════════════════════════════════════════════
#  View: Simulation starten
# ════════════════════════════════════════════════════════════════════════
class SimFrame(tk.Frame):
    ephemeral = True
    view_title = "Simulation starten"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Simulation starten",
                   "MuJoCo + Whole-Body-Policy. Alle Optionen in einem Fenster.")

        # Aktionsleiste unten (ausserhalb des Scrollbereichs).
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", side="bottom", padx=16, pady=12)
        primary_button(foot, "▶  Simulation starten", self._start, GREEN).pack(side="left")
        tk.Button(foot, text="Abbrechen (Menue)", command=app.show_menu, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=8).pack(side="right")

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        # Defaults spiegeln start.sh (Sim-Zweig).
        self.v_rviz = tk.BooleanVar(value=False)
        self.v_hands = tk.BooleanVar(value=False)
        self.v_open_guis = tk.BooleanVar(value=True)
        self.v_nav = tk.BooleanVar(value=False)
        self.v_rebuild = tk.BooleanVar(value=False)
        self.v_env = tk.StringVar(value=DEFAULT_ENV_LABEL)
        self.v_rt = tk.StringVar(value="1.0")

        s = section(b, "Umgebung (G1 bleibt gleich, nur die Welt wechselt)")
        row = tk.Frame(s, bg=CARD)
        row.pack(fill="x", pady=2)
        self._env_box = ttk.Combobox(row, textvariable=self.v_env,
                                     state="readonly", width=44)
        self._env_box.pack(side="left")
        tk.Button(row, text="↻", command=self._reload_envs, bg=BG, fg=FG,
                  relief="flat", padx=8).pack(side="left", padx=6)
        self._env_hint = tk.Label(
            s, text="", bg=CARD, fg=MUTED, font=("TkDefaultFont", 9),
            justify="left", wraplength=560)
        self._env_hint.pack(anchor="w", pady=(2, 0))
        self._reload_envs()

        s = section(b, "Visualisierung & Features")
        toggle_row(s, "RViz mitstarten", self.v_rviz,
                   "CoM-/TF-Visualisierung (MuJoCo-Fenster kommt immer)")
        toggle_row(s, "Navigation mitstarten", self.v_nav,
                   "dijkstra_planner + nav2point + Sim-Glue")

        s = section(b, "Inspire-FTP-Haende")
        toggle_row(s, "Inspire-Haende (Finger steuerbar + GUIs)", self.v_hands,
                   "sonst Rubber-Hand")
        self._open_row = toggle_row(s, "Hand-GUIs automatisch im Browser oeffnen",
                                    self.v_open_guis)
        self.v_hands.trace_add("write", lambda *_: self._sync_hands())
        self._sync_hands()

        s = section(b, "Erweitert")
        field_row(s, "Realtime-Faktor (1.0=Echtzeit)", self.v_rt, width=8)
        toggle_row(s, "Docker-Images vor dem Start neu bauen (--build)", self.v_rebuild,
                   "nach Code-/Dockerfile-Aenderungen")

    def _reload_envs(self) -> None:
        """Umgebungsliste frisch aus scenes/ ziehen (Editor laeuft ggf. parallel)."""
        self._scene_paths: dict[str, str] = {DEFAULT_ENV_LABEL: ""}
        names = [DEFAULT_ENV_LABEL]
        for env in list_scenes():
            names.append(env.name)
            self._scene_paths[env.name] = env.name
        self._env_box.configure(values=names)
        if self.v_env.get() not in names:
            self.v_env.set(DEFAULT_ENV_LABEL)

        hint = "Umgebungen anlegen/bearbeiten: Menue -> 'Umgebungen bearbeiten'."
        broken = list_broken_scenes()
        if broken:
            hint += ("\nNicht verwendbar: "
                     + ", ".join(f"{p.name} ({why})" for p, why in broken))
        self._env_hint.configure(text=hint)

    def _sync_hands(self) -> None:
        state = "normal" if self.v_hands.get() else "disabled"
        for child in self._open_row.winfo_children():
            try:
                child.configure(state=state)
            except tk.TclError:
                pass

    def _prepare_env(self, name: str) -> bool:
        """Kombinierte Szene vor dem Start erzeugen und Fehler klar melden."""
        if env_store.find(name) is None:
            messagebox.showerror(
                "Umgebung weg",
                f"Die Umgebung '{name}' gibt es nicht mehr in\n{SCENES_DIR}.\n\n"
                "Liste mit ↻ aktualisieren.", parent=self)
            self._reload_envs()
            return False
        ok, out = build_env_scene(name, self.v_hands.get())
        if not ok:
            messagebox.showerror(
                "Umgebung laesst sich nicht laden",
                f"Die Umgebung '{name}' konnte nicht mit dem G1 kombiniert werden:\n\n"
                f"{out or '(keine Meldung)'}\n\n"
                "Entweder die Umgebung im Editor korrigieren oder 'Standard' waehlen.",
                parent=self)
            return False
        return True

    def _start(self) -> None:
        env_name = self._scene_paths.get(self.v_env.get(), "")
        if env_name and not self._prepare_env(env_name):
            return

        env = os.environ.copy()
        env["G1_MODE"] = "sim"
        env["USE_RVIZ"] = "true" if self.v_rviz.get() else "false"
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        env["OPEN_GUIS"] = "true" if (self.v_hands.get() and self.v_open_guis.get()) else "false"
        env["G1_ENABLE_NAV"] = "1" if self.v_nav.get() else "0"
        env["G1_ENV"] = env_name
        rt = self.v_rt.get().strip() or "1.0"
        try:
            float(rt)
        except ValueError:
            messagebox.showerror("Ungueltig", "Realtime-Faktor muss eine Zahl sein (z.B. 1.0).",
                                 parent=self)
            return
        env["SIM_REALTIME_FACTOR"] = rt

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")
        stop_cmd = ["docker", "compose", "--profile", "sim", "down", "--remove-orphans"]
        self.app.start_process("Simulation", argv, cwd=HERE, env=env,
                               stop_cmd=stop_cmd,
                               stop_note="Stoppt den Sim-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  View: Echten Roboter starten
# ════════════════════════════════════════════════════════════════════════
class RealFrame(tk.Frame):
    ephemeral = True
    view_title = "Echten Roboter starten"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Echten Roboter starten",
                   "Unitree-Loco + Arme + Haende ueber LAN. Der Roboter bewegt sich!", RED)

        # Defaults spiegeln start.sh (Real-Zweig).
        self.v_hands = tk.BooleanVar(value=True)
        self.v_open_guis = tk.BooleanVar(value=True)
        self.v_rviz = tk.BooleanVar(value=True)
        self.v_lidar = tk.BooleanVar(value=False)
        self.v_rebuild = tk.BooleanVar(value=False)
        self.v_left = tk.StringVar(value="192.168.123.210")
        self.v_right = tk.StringVar(value="192.168.123.211")
        self.v_port = tk.StringVar(value="6000")
        self.v_vx = tk.StringVar(value="0.4")
        self.v_vy = tk.StringVar(value="0.3")
        self.v_vyaw = tk.StringVar(value="0.4")
        self.v_joy = tk.StringVar(value="Wireless Controller")
        self.v_confirm = tk.BooleanVar(value=False)

        # Aktionsleiste unten.
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", side="bottom", padx=16, pady=12)
        self.start_btn = primary_button(foot, "🤖  Echten Roboter starten", self._start, RED)
        self.start_btn.configure(state="disabled")
        self.start_btn.pack(side="left")
        tk.Button(foot, text="Abbrechen (Menue)", command=app.show_menu, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=8).pack(side="right")

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        s = section(b, "Netzwerk-Interface zum G1 (Roboter-LAN = 192.168.123.x)")
        nics = detect_nics()
        values = []
        default = ""
        self._iface_map: dict[str, str] = {}
        for name, ip in nics:
            label = f"{name}  ({ip or 'keine IPv4'})"
            values.append(label)
            self._iface_map[label] = name
            if ip.startswith("192.168.123."):
                default = label
        if not values:
            values = [""]
        self.v_iface = tk.StringVar(value=default or values[0])
        ttk.Combobox(s, textvariable=self.v_iface, values=values, width=42).pack(anchor="w", pady=2)
        tk.Label(s, text="Kein passendes Interface? Namen direkt eintippen (z.B. enp3s0).",
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9)).pack(anchor="w")

        s = section(b, "Inspire-FTP-Haende (Modbus TCP im Roboter-LAN)")
        toggle_row(s, "Haende ansteuern (Hand-Bridge + GUIs)", self.v_hands)
        self._hand_box = tk.Frame(s, bg=CARD)
        self._hand_box.pack(fill="x")
        field_row(self._hand_box, "IP linke Hand", self.v_left)
        field_row(self._hand_box, "IP rechte Hand", self.v_right)
        field_row(self._hand_box, "Modbus-Port", self.v_port, width=8)
        toggle_row(self._hand_box, "Hand-GUIs automatisch oeffnen", self.v_open_guis)
        self.v_hands.trace_add("write", lambda *_: self._sync_hands())
        self._sync_hands()

        s = section(b, "Steuerung & Visualisierung")
        toggle_row(s, "RViz mitstarten (IK-Marker der Arm-Manipulation)", self.v_rviz)
        field_row(s, "Joystick-Name (evdev)", self.v_joy, width=24)
        toggle_row(s, "LiDAR aktivieren (G1_ENABLE_LIDAR)", self.v_lidar,
                   "nur mit Livox-Setup")

        s = section(b, "Walk-Limits (konservativ fuer erste Tests)")
        row = tk.Frame(s, bg=CARD)
        row.pack(fill="x", pady=2)
        for lbl, var in (("vx", self.v_vx), ("vy", self.v_vy), ("vyaw", self.v_vyaw)):
            tk.Label(row, text=lbl, bg=CARD, fg=FG).pack(side="left", padx=(0, 2))
            tk.Entry(row, textvariable=var, width=6, bg=BG, fg=FG,
                     insertbackground=FG, relief="flat").pack(side="left", padx=(0, 12), ipady=2)

        s = section(b, "Erweitert")
        toggle_row(s, "Docker-Images vor dem Start neu bauen (--build)", self.v_rebuild)

        gate = tk.Frame(b, bg="#3a2326", bd=0)
        gate.pack(fill="x", padx=14, pady=8)
        tk.Label(gate, text="⚠  ACHTUNG: ECHTER ROBOTER", bg="#3a2326", fg=RED,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        tk.Label(gate,
                 text=("Kein Auto-Start: der Roboter bewegt sich erst nach Streamdeck-\n"
                       "Kommandos. E-STOP = Damp = Roboter sackt ZUSAMMEN (sichern!).\n"
                       "Checkliste vorher: g1pilot/docs/70_echtroboter_anleitung.md"),
                 bg="#3a2326", fg=FG, justify="left",
                 font=("TkDefaultFont", 9)).pack(anchor="w", padx=12, pady=(2, 6))
        tk.Checkbutton(gate,
                       text="Ich habe die Sicherheitshinweise gelesen und der Bereich ist frei.",
                       variable=self.v_confirm, bg="#3a2326", fg=FG, selectcolor=BG,
                       activebackground="#3a2326", activeforeground=FG,
                       command=self._sync_confirm).pack(anchor="w", padx=12, pady=(0, 8))

    def _sync_hands(self) -> None:
        state = "normal" if self.v_hands.get() else "disabled"
        for child in self._hand_box.winfo_children():
            for w in child.winfo_children():
                try:
                    w.configure(state=state)
                except tk.TclError:
                    pass

    def _sync_confirm(self) -> None:
        self.start_btn.configure(state="normal" if self.v_confirm.get() else "disabled")

    def _start(self) -> None:
        if not self.v_confirm.get():
            return
        iface_label = self.v_iface.get().strip()
        iface = self._iface_map.get(iface_label, iface_label.split()[0] if iface_label else "")
        if not iface:
            messagebox.showerror("Interface fehlt",
                                 "Bitte ein Netzwerk-Interface waehlen oder eintippen.",
                                 parent=self)
            return

        env = os.environ.copy()
        env["G1_MODE"] = "real"
        env["G1_REAL_CONFIRM"] = "1"  # Sicherheits-Gate von start.sh im --yes-Modus
        env["ROBOT_INTERFACE"] = iface
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        env["OPEN_GUIS"] = "true" if (self.v_hands.get() and self.v_open_guis.get()) else "false"
        env["G1_HAND_LEFT_HOST"] = self.v_left.get().strip() or "192.168.123.210"
        env["G1_HAND_RIGHT_HOST"] = self.v_right.get().strip() or "192.168.123.211"
        env["G1_HAND_PORT"] = self.v_port.get().strip() or "6000"
        env["USE_RVIZ"] = "true" if self.v_rviz.get() else "false"
        env["G1_ENABLE_LIDAR"] = "1" if self.v_lidar.get() else "0"
        env["JOYSTICK_NAME"] = self.v_joy.get().strip() or "Wireless Controller"
        env["G1_MAX_VX"] = self.v_vx.get().strip() or "0.4"
        env["G1_MAX_VY"] = self.v_vy.get().strip() or "0.3"
        env["G1_MAX_VYAW"] = self.v_vyaw.get().strip() or "0.4"

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")
        stop_cmd = ["docker", "compose", "--profile", "real", "down", "--remove-orphans"]
        self.app.start_process("Echter Roboter", argv, cwd=HERE, env=env,
                               stop_cmd=stop_cmd,
                               stop_note="Stoppt den Real-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  View: Umgebungen bearbeiten (scene_editor)
# ════════════════════════════════════════════════════════════════════════
class SceneFrame(tk.Frame):
    ephemeral = True
    view_title = "Umgebungen bearbeiten"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Umgebungen bearbeiten",
                   "Umgebungen aus scene_editor/scenes/<name>/ — dieselben, die beim "
                   "Sim-Start waehlbar sind.")
        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        self._body = body.body
        self.v_hands = tk.BooleanVar(value=False)
        if not scene_editor_ready():
            self._show_setup_needed()
            return
        self._build_editor_ui()

    def _show_setup_needed(self) -> None:
        box = section(self._body, "Einmaliges Setup noetig")
        tk.Label(box, text="Der Scene-Editor braucht ein virtualenv (scene_editor/.venv).\n"
                           "Das wird einmalig eingerichtet (Internet noetig, danach gecacht).",
                 bg=CARD, fg=FG, justify="left").pack(anchor="w", pady=(0, 8))
        primary_button(box, "Setup jetzt ausfuehren", self._run_setup, ACCENT).pack(anchor="w")

    def _run_setup(self) -> None:
        self.app.start_process("Scene-Editor Setup", ["bash", str(SETUP_SH)],
                               cwd=SCENE_DIR, env=os.environ.copy(),
                               stop_note="Bricht das Setup ab.")

    def _build_editor_ui(self) -> None:
        s = section(self._body, "Vorhandene Umgebungen")
        listwrap = tk.Frame(s, bg=CARD)
        listwrap.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(listwrap, height=8, bg=BG, fg=FG,
                                  selectbackground=ACCENT, relief="flat",
                                  activestyle="none", exportselection=False)
        sb = ttk.Scrollbar(listwrap, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=sb.set)
        self.listbox.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self._problem_label = tk.Label(
            s, text="", bg=CARD, fg=AMBER, font=("TkDefaultFont", 9),
            justify="left", wraplength=560)
        self._problem_label.pack(anchor="w", pady=(4, 0))
        self._reload_scenes()

        toggle_row(s, "Beim Ansehen 'mit G1' die Inspire-Haende laden", self.v_hands)

        act = section(self._body, "Aktion fuer die gewaehlte Umgebung")
        grid = tk.Frame(act, bg=CARD)
        grid.pack(fill="x")
        primary_button(grid, "✎  Im Editor bearbeiten",
                       lambda: self._run_scene("edit"), ACCENT).grid(
            row=0, column=0, padx=4, pady=4, sticky="ew")
        primary_button(grid, "👁  Nur Umgebung ansehen",
                       lambda: self._run_scene("view"), CARD).grid(
            row=0, column=1, padx=4, pady=4, sticky="ew")
        primary_button(grid, "🤖  Mit G1 ansehen",
                       lambda: self._run_scene("with-g1"), GREEN).grid(
            row=1, column=0, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="✔  Auf Ladbarkeit pruefen", command=self._check_scene,
                  bg=CARD, fg=FG, relief="flat", padx=10, pady=8).grid(
            row=1, column=1, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="↻  Liste aktualisieren", command=self._reload_scenes,
                  bg=CARD, fg=FG, relief="flat", padx=10, pady=8).grid(
            row=2, column=0, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="🗑  Umgebung loeschen", command=self._delete_scene,
                  bg=CARD, fg=RED, relief="flat", padx=10, pady=8).grid(
            row=2, column=1, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="📦  Als Zip exportieren", command=self._pack_scene,
                  bg=CARD, fg=FG, relief="flat", padx=10, pady=8).grid(
            row=3, column=0, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="📥  Zip einspielen", command=self._unpack_scene,
                  bg=CARD, fg=FG, relief="flat", padx=10, pady=8).grid(
            row=3, column=1, padx=4, pady=4, sticky="ew")
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        new = section(self._body, "Neue Umgebung")
        newrow = tk.Frame(new, bg=CARD)
        newrow.pack(fill="x")
        primary_button(newrow, "＋  Leere Umgebung im Editor",
                       lambda: self._run_cmd(["new"], "Neue Umgebung", browser=True),
                       ACCENT).pack(side="left", padx=(0, 6))
        tk.Button(newrow, text="✨  Aus Text-Prompt generieren", command=self._run_prompt,
                  bg=CARD, fg=FG, relief="flat", padx=12, pady=8).pack(side="left")
        tk.Label(new, text=(
                     f"Der Editor oeffnet einen lokalen Webserver ({EDITOR_URL}) im Browser. "
                     "Zum Speichern oben im Editor unter 'Umgebung speichern' nur den NAMEN "
                     "eintippen — die Umgebung landet als Ordner in scenes/<name>/ (mit allen "
                     "Meshes) und steht danach beim Sim-Start zur Auswahl (Liste hier mit ↻ "
                     "auffrischen). Weitergeben: 📦 Zip exportieren / 📥 Zip einspielen."),
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9),
                 wraplength=560, justify="left").pack(anchor="w", pady=(6, 0))

    def _reload_scenes(self) -> None:
        """Liste neu aufbauen und die vorherige Auswahl beibehalten."""
        previous = None
        scenes = getattr(self, "_scenes", None)
        sel = self.listbox.curselection() if scenes else ()
        if scenes and sel:
            previous = scenes[sel[0]].name

        self.listbox.configure(state="normal")
        self.listbox.delete(0, "end")
        self._scenes = list_scenes()
        if not self._scenes:
            self.listbox.insert("end", "(noch keine Umgebungen — 'Leere Umgebung' anlegen)")
            self.listbox.configure(state="disabled")
        else:
            for env in self._scenes:
                label = env.name + ("   (altes Format)" if env.legacy else "")
                self.listbox.insert("end", label)
            idx = 0
            for i, env in enumerate(self._scenes):
                if env.name == previous:
                    idx = i
                    break
            self.listbox.selection_set(idx)
            self.listbox.see(idx)

        broken = list_broken_scenes()
        if broken:
            self._problem_label.configure(
                text="Nicht als Umgebung verwendbar (werden ausgeblendet):\n"
                     + "\n".join(f"   • {p.name} — {why}" for p, why in broken))
        else:
            self._problem_label.configure(text="")

    def _selected_scene(self):
        if not getattr(self, "_scenes", None):
            messagebox.showinfo(
                "Keine Umgebung da",
                "In scene_editor/scenes/ liegt noch keine Umgebung.\n"
                "Unten mit '＋ Leere Umgebung im Editor' eine anlegen.", parent=self)
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("Keine Auswahl", "Bitte zuerst eine Umgebung waehlen.",
                                parent=self)
            return None
        scene = self._scenes[sel[0]]
        if not scene.xml.is_file():
            messagebox.showerror("Umgebung weg",
                                 f"'{scene.name}' gibt es nicht mehr.", parent=self)
            self._reload_scenes()
            return None
        return scene

    def _check_scene(self) -> None:
        """Umgebung mit dem G1 kombinieren, ohne etwas zu starten."""
        scene = self._selected_scene()
        if scene is None:
            return
        ok, out = build_env_scene(scene.name, self.v_hands.get())
        missing = [] if scene.legacy else env_store.containment_problems(scene.xml)
        if ok and not missing:
            messagebox.showinfo(
                "Umgebung in Ordnung",
                f"'{scene.name}' laesst sich mit dem G1 kombinieren und laden.\n\n"
                f"{out}", parent=self)
        elif ok:
            messagebox.showwarning(
                "Umgebung laedt, ist aber nicht eigenstaendig",
                f"'{scene.name}' laedt, braucht aber Dateien ausserhalb ihres Ordners "
                "(Zip/Weitergabe wuerde nicht funktionieren):\n\n"
                + "\n".join(f"• {m}" for m in missing)
                + "\n\nEinmal im Editor oeffnen und speichern behebt das.", parent=self)
        else:
            messagebox.showerror(
                "Umgebung fehlerhaft",
                f"'{scene.name}' laesst sich nicht laden:\n\n{out or '(keine Meldung)'}",
                parent=self)

    def _delete_scene(self) -> None:
        scene = self._selected_scene()
        if scene is None:
            return
        what = scene.xml if scene.legacy else scene.folder
        if not messagebox.askyesno(
                "Umgebung loeschen",
                f"'{scene.name}' wirklich loeschen?\n\n{what}\n"
                + ("(die zugehoerige .json-Datei wird mitgeloescht)" if scene.legacy
                   else "(der ganze Ordner samt Meshes)"), parent=self):
            return
        try:
            if scene.legacy:
                scene.xml.unlink()
                sidecar = scene.xml.with_suffix(".json")
                if sidecar.is_file():
                    sidecar.unlink()
            else:
                shutil.rmtree(scene.folder)
        except OSError as exc:
            messagebox.showerror("Loeschen fehlgeschlagen", str(exc), parent=self)
            return
        self._reload_scenes()

    def _pack_scene(self) -> None:
        """Umgebung als Zip speichern (zum Weitergeben/Sichern)."""
        scene = self._selected_scene()
        if scene is None:
            return
        EXPORT_DIR.mkdir(parents=True, exist_ok=True)
        target = filedialog.asksaveasfilename(
            parent=self, title=f"'{scene.name}' als Zip speichern",
            initialdir=str(EXPORT_DIR), initialfile=f"{scene.name}.zip",
            defaultextension=".zip", filetypes=[("Zip", "*.zip")])
        if not target:
            return
        try:
            if scene.legacy:
                env_store.migrate(scene)
            out = env_store.pack(scene.name, target)
        except (env_store.EnvError, OSError) as exc:
            messagebox.showerror("Export fehlgeschlagen", str(exc), parent=self)
            return
        self._reload_scenes()
        messagebox.showinfo(
            "Umgebung exportiert",
            f"{out}\n({out.stat().st_size / 1e6:.1f} MB)\n\nAuf dem anderen Rechner hier "
            "mit '📥 Zip einspielen' oder  ./launch.sh unpack <zip>.", parent=self)

    def _unpack_scene(self) -> None:
        """Umgebung aus einem Zip einspielen."""
        path = filedialog.askopenfilename(
            parent=self, title="Umgebung (Zip) einspielen",
            filetypes=[("Zip", "*.zip"), ("Alle Dateien", "*")])
        if not path:
            return
        try:
            env = env_store.unpack(path)
        except env_store.EnvError as exc:
            if "gibt es schon" not in str(exc):
                messagebox.showerror("Einspielen fehlgeschlagen", str(exc), parent=self)
                return
            if not messagebox.askyesno(
                    "Umgebung ersetzen?",
                    f"{exc}\n\nVorhandene Umgebung durch das Zip ersetzen?", parent=self):
                return
            try:
                env = env_store.unpack(path, force=True)
            except (env_store.EnvError, OSError) as exc2:
                messagebox.showerror("Einspielen fehlgeschlagen", str(exc2), parent=self)
                return
        except OSError as exc:
            messagebox.showerror("Einspielen fehlgeschlagen", str(exc), parent=self)
            return
        self._reload_scenes()
        messagebox.showinfo("Umgebung eingespielt",
                            f"'{env.name}' liegt jetzt in {env.folder}\n"
                            "und ist beim Sim-Start waehlbar.", parent=self)

    def _run_scene(self, cmd: str) -> None:
        scene = self._selected_scene()
        if scene is None:
            return
        env = os.environ.copy()
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        title = {"edit": "Editor", "view": "Viewer", "with-g1": "Viewer + G1"}[cmd]
        # 'edit' startet den Web-Editor -> Browser oeffnen; view/with-g1 sind
        # native MuJoCo-Fenster.
        self._run_cmd([cmd, str(scene.xml)], f"{title}: {scene.name}", env=env,
                      browser=(cmd == "edit"))

    def _run_prompt(self) -> None:
        text = simpledialog.askstring(
            "Umgebung aus Text",
            "Beschreibe die Umgebung (Englisch funktioniert am besten):",
            parent=self)
        if not text or not text.strip():
            return
        if not os.environ.get("OPENAI_API_KEY"):
            messagebox.showerror(
                "OpenAI-Key fehlt",
                "Umgebungen aus Text brauchen einen OpenAI-API-Key.\n\n"
                "Vor dem Start setzen:\n    export OPENAI_API_KEY=sk-...\n\n"
                "Ohne Key: '＋ Leere Umgebung im Editor' und die Objekte selbst setzen.",
                parent=self)
            return
        self._run_cmd(["prompt", text.strip()], "Umgebung generieren", browser=True)

    def _run_cmd(self, args: list[str], title: str, env: dict | None = None,
                 browser: bool = False) -> None:
        env = env or os.environ.copy()
        env.setdefault("SCENE_EDITOR_PORT", EDITOR_PORT)
        self.app.start_process(f"Scene-Editor — {title}",
                               ["bash", str(LAUNCH_SH), *args],
                               cwd=SCENE_DIR, env=env,
                               stop_note="Beendet den Editor/Viewer-Prozess.",
                               browser_url=EDITOR_URL if browser else None)


# ════════════════════════════════════════════════════════════════════════
#  View: Hauptmenue
# ════════════════════════════════════════════════════════════════════════
class MenuFrame(tk.Frame):
    ephemeral = True
    view_title = "Startmenue"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app

        head = tk.Frame(self, bg=BG)
        head.pack(fill="x", padx=24, pady=(20, 4))
        tk.Label(head, text="G1 Robot Control", bg=BG, fg=FG,
                 font=("TkDefaultFont", 20, "bold")).pack(anchor="w")
        tk.Label(head, text="Simulation, echter Roboter und Umgebungen — alles in einem Fenster.",
                 bg=BG, fg=MUTED, font=("TkDefaultFont", 11)).pack(anchor="w")

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        wrap = tk.Frame(b, bg=BG)
        wrap.pack(fill="x", padx=24, pady=8)
        big_button(wrap, "▶", "Simulation starten",
                   "MuJoCo + Whole-Body-Policy — gefahrlos testen", GREEN, app.open_sim)
        big_button(wrap, "🤖", "Echten Roboter starten",
                   "G1 per LAN — Loco + Arme + Haende (Roboter bewegt sich!)", RED, app.open_real)
        big_button(wrap, "🏗", "Umgebungen bearbeiten",
                   "Szenen anlegen, bearbeiten und mit dem G1 ansehen", ACCENT, app.open_scenes)

        # Laufende / letzte Prozesse (damit nichts 'verloren' geht).
        if app.consoles:
            s = section(b, "Laufende / letzte Prozesse")
            for c in app.consoles:
                row = tk.Frame(s, bg=CARD)
                row.pack(fill="x", pady=2)
                dot = GREEN if c.is_running() else MUTED
                tk.Label(row, text="●", bg=CARD, fg=dot).pack(side="left", padx=(0, 6))
                tk.Label(row, text=f"{c.view_title}  ({c.state()})", bg=CARD, fg=FG,
                         anchor="w").pack(side="left")
                tk.Button(row, text="Ansehen", command=lambda x=c: app.show_console(x),
                          bg=BG, fg=FG, relief="flat", padx=10, pady=3).pack(side="right", padx=4)
                if c.is_running():
                    tk.Button(row, text="Stoppen", command=lambda x=c: self._stop(x),
                              bg=RED, fg="white", relief="flat", padx=10,
                              pady=3).pack(side="right", padx=4)
                else:
                    tk.Button(row, text="Entfernen", command=lambda x=c: app.remove_console(x),
                              bg=BG, fg=MUTED, relief="flat", padx=10,
                              pady=3).pack(side="right", padx=4)

        # Sekundaerzeile.
        sec = tk.Frame(b, bg=BG)
        sec.pack(fill="x", padx=24, pady=(6, 12))
        tk.Button(sec, text="■  Laufende Stacks stoppen", command=app.stop_all,
                  bg=CARD, fg=FG, relief="flat", padx=12, pady=6).pack(side="left")
        docs_btn = ttk.Menubutton(sec, text="Dokumentation")
        docmenu = tk.Menu(docs_btn, tearoff=0)
        for label, path in DOCS:
            docmenu.add_command(label=label, command=lambda p=path: open_path(p))
        docs_btn["menu"] = docmenu
        docs_btn.pack(side="left", padx=8)
        tk.Button(sec, text="Beenden", command=app._quit, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=6).pack(side="right")

    def _stop(self, console) -> None:
        console.stop()
        # Menue neu aufbauen, damit der Status aktualisiert wird.
        self.app.show_menu()


# ════════════════════════════════════════════════════════════════════════
#  App: ein Fenster, Views werden im Inhalt ausgetauscht (Router)
# ════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("G1 — Startmenue")
        self.configure(bg=BG)
        self.geometry("900x760")
        self.minsize(700, 600)

        self._quitting = False
        self._current = None
        self.consoles: list[ConsoleFrame] = []

        # Statuszeile unten (bleibt ueber allen Views sichtbar).
        self.status = tk.Label(self, text="", bg="#171a20", fg=MUTED, anchor="w",
                               font=("TkFixedFont", 9))
        self.status.pack(fill="x", side="bottom")

        # Container fuellt den Rest; hier werden die Views ein-/ausgeblendet.
        self.container = tk.Frame(self, bg=BG)
        self.container.pack(fill="both", expand=True)

        self.protocol("WM_DELETE_WINDOW", self._quit)
        self.show_menu()
        self.after(200, self.refresh_status)

    # ── Router ────────────────────────────────────────────────────────────
    def show(self, frame) -> None:
        if self._current is frame:
            return
        if self._current is not None:
            self._current.pack_forget()
            if getattr(self._current, "ephemeral", False):
                self._current.destroy()
        self._current = frame
        frame.pack(fill="both", expand=True)
        self.title(f"G1 — {getattr(frame, 'view_title', 'Startmenue')}")

    def show_menu(self) -> None:
        self.show(MenuFrame(self.container, self))

    def open_sim(self) -> None:
        if self._docker_guard():
            self.show(SimFrame(self.container, self))

    def open_real(self) -> None:
        if self._docker_guard():
            self.show(RealFrame(self.container, self))

    def open_scenes(self) -> None:
        self.show(SceneFrame(self.container, self))

    def show_console(self, console: ConsoleFrame) -> None:
        if console not in self.consoles:
            self.consoles.append(console)
        self.show(console)

    def notify_finished(self, console: ConsoleFrame, *, user_stopped: bool) -> None:
        """Ein Prozess ist fertig (sauber gestoppt oder von selbst beendet).

        - Wird er gerade angesehen UND wurde vom Nutzer gestoppt (MuJoCo/Container
          sind dann sauber zu): automatisch zurueck ins Startmenue.
        - Ist gerade das Menue offen: nur die Prozessliste auffrischen.
        - Sonst (andere View sichtbar): nichts tun, nicht wegreissen. Von selbst
          beendete/abgestuerzte Prozesse bleiben in ihrer View, damit Fehler/Log
          lesbar bleiben.
        """
        if self._current is console:
            if user_stopped:
                self.show_menu()
        elif isinstance(self._current, MenuFrame):
            self.show_menu()

    def start_process(self, title, argv, *, cwd, env=None, stop_cmd=None,
                      stop_note="", browser_url=None) -> ConsoleFrame:
        console = ConsoleFrame(self.container, self, title, argv, cwd=cwd, env=env,
                               stop_cmd=stop_cmd, stop_note=stop_note,
                               browser_url=browser_url)
        self.consoles.append(console)
        self.show(console)
        return console

    def remove_console(self, console: ConsoleFrame) -> None:
        if console in self.consoles:
            self.consoles.remove(console)
        if self._current is console:
            self._current = None
        console.destroy()
        self.show_menu()

    # ── Aktionen ──────────────────────────────────────────────────────────
    def _docker_guard(self) -> bool:
        if docker_ready():
            return True
        return messagebox.askyesno(
            "Docker nicht erreichbar",
            "Docker scheint nicht zu laufen oder ist nicht installiert.\n"
            "Trotzdem fortfahren?")

    def stop_all(self) -> None:
        if not messagebox.askyesno("Stoppen",
                                   "Sim- UND Real-Stack stoppen (docker compose down)?"):
            return
        cmd = ["bash", "-c",
               "docker compose --profile sim down --remove-orphans; "
               "docker compose --profile real down --remove-orphans"]
        self.start_process("Stacks stoppen", cmd, cwd=HERE, env=os.environ.copy())

    def _quit(self) -> None:
        # Ein Fenster, ein Schliessen: beendet den Launcher komplett. Bereits
        # gestartete Stacks laufen unabhaengig im Docker weiter (bewusst).
        # Nur der Web-Editor wird mitbeendet (siehe ConsoleFrame.terminate_editor).
        self._quitting = True
        for console in list(self.consoles):
            console.terminate_editor()
        try:
            self.destroy()
        finally:
            # Garantiert beenden, auch wenn noch Hintergrund-Threads/Subprozesse
            # (Log-Reader, docker compose) haengen.
            os._exit(0)

    def refresh_status(self) -> None:
        editor = "Scene-Editor: bereit" if scene_editor_ready() else "Scene-Editor: Setup noetig"
        scenes = f"Umgebungen: {len(list_scenes())}"
        self.status.configure(text=f"  Docker: pruefe…   |   {editor}   |   {scenes}")

        def worker():
            docker = "Docker: ok" if docker_ready() else "Docker: nicht erreichbar"
            text = f"  {docker}   |   {editor}   |   {scenes}"
            if self._quitting:
                return
            try:
                self.after(0, lambda: self.status.configure(text=text))
            except (tk.TclError, RuntimeError):
                pass

        threading.Thread(target=worker, daemon=True).start()


def main() -> int:
    app = App()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
