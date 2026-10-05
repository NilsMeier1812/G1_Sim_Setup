# Scene Editor – MuJoCo-Umgebungen einfach bauen

Kleiner Baukasten, um MuJoCo-Umgebungen fuer den G1 zu bauen: Objekte per
Maus platzieren, eigene **Meshes (STL/OBJ/PLY/GLB/...) und CAD-Dateien
(STEP/IGES) importieren** – auch ganze Anlagen/Roboterzellen – und das Ergebnis
als MuJoCo-XML exportieren. Grundlage ist der
[`mujoco-scene-editor`](https://github.com/markusgrotz/mujoco-scene-editor)
(browserbasierter Editor) plus zwei fertige Beispiel-Szenen und ein paar
Helfer-Skripte.

> Ergaenzt das vorhandene `terrain_tool/` (Terrain per Python-Skript). Der
> Scene Editor ist der **visuelle** Weg: Objekte/Meshes per Maus setzen.

> **Jede Umgebung ist ein Ordner:** `scene_editor/scenes/<name>/` mit
> `umgebung.xml` und allen Meshes, die sie braucht, in `meshes/` daneben. Jeder
> solche Ordner ist eine Umgebung – waehlbar **im Editor** (`./launch.sh`) UND
> **beim G1-Start** (`g1pilot/start.sh`, Sim). Ordner kopieren, zippen oder
> einchecken = Umgebung weitergeben (siehe Abschnitt 5).

## Konzept: Basis + Umgebung

Das System trennt **Basis** (immer gleich) von **Umgebung** (wechselt):

| | kommt aus | Inhalt |
|---|---|---|
| **Basis** (immer automatisch) | `build_env_scene.py` | G1-Roboter, **Lichtquelle**, Boden, **Weld** (haelt den G1 anfangs fest), visual/statistic – die technischen Grunddinge |
| **Umgebung** (waehlbar) | `scenes/<name>/umgebung.xml` (+ `meshes/`) | **nur Hindernisse / Objekte zum Interagieren/Greifen** |

Du baust in einer Umgebung also **nur die Objekte**. G1, Licht, Boden und Weld
werden beim Laden immer automatisch dazugefuegt – nie in die Umgebung schreiben.
(Legt der Editor doch mal einen eigenen Boden/eine Lichtquelle an, wirft der
Generator sie beim Kombinieren automatisch raus.)

### Hindernis vs. Greif-Objekt (fuer Nav/IK)

Objekte werden nach RViz uebertragen (siehe `g1pilot/docs/51_navigation_technik.md`) und dort
in zwei Klassen unterschieden — **Hindernis** (der Arm weicht aus) und
**Greif-Objekt** (die Hand darf ran, das Objekt bewegt sich beim Anfassen mit).
Klassifikation per **Namenskonvention**: benennst du ein Objekt mit dem
Praefix **`grasp_`** (Gross-/Kleinschreibung egal, z.B. `grasp_apfel`), macht
`build_env_scene.py` beim Kombinieren automatisch einen **freien, beweglichen
Koerper** daraus (`<freejoint/>`) — nur so kann MuJoCo es beim Greifen/Anfassen
bewegen. Alle anderen Objekte werden zu statischen Hindernissen.

**Im Editor:** Objekt oben unter „Elements" waehlen, dann im Ordner
**„Objekt umbenennen"** entweder **„Greif-Objekt an/aus (grasp_)"** klicken
(setzt/entfernt das Praefix) oder einen Namen eintippen und **„Umbenennen"**.
Danach **„Speichern"**. (Der eingebaute Editor kann nicht umbenennen – der
Ordner wird von `run_editor.py` ergaenzt.)

**Startpunkt des G1:** Standardmaessig startet der Roboter im Ursprung (0,0),
Blick in +x. Soll er woanders stehen (z.B. vor statt im Tisch), ein beliebiges
Objekt (z.B. eine Box) dorthin setzen und es **`g1_spawn`** nennen (Ordner
„Objekt umbenennen"). Position = Startpunkt, Drehung um die Hochachse =
Blickrichtung. Die Markierung wird beim Kombinieren entfernt (kein Hindernis);
auch START / START BALANCING stellen den Roboter wieder dort auf.

```
box_demo        -> Hindernis (statisch, der Arm weicht aus)
grasp_apfel      -> Greif-Objekt (beweglich, die Hand darf ran)
```

> **Der Name entscheidet, nicht der Editor-Export.** Der Editor haengt jedes per
> Maus gesetzte Objekt in einen eigenen Koerper mit freiem Gelenk — ohne Umbau
> wuerde die halbe Umgebung beim Start umfallen. `build_env_scene.py`
> normalisiert deshalb beim Kombinieren **alles** auf die kanonische Form
> (Hindernis = statisches `<geom>`, Greif-Objekt = `<body>` mit `<freejoint/>`),
> allein anhand des Namens. Du musst also nichts von Hand nachbauen — nur
> sinnvoll benennen.

---

## Was ist drin?

```
scene_editor/
├── setup.sh                    # einmaliges Setup (virtualenv + Installation)
├── launch.sh                   # Menue / Editor / Viewer starten
├── run_editor.py               # Editor-Start + Speichern-nur-mit-Name, Umbenennen, Import
├── cad_import.py               # CAD/Mesh -> Einzelteil-Meshes + Umgebung (auch als CLI)
├── env_store.py                # Umgebungs-Ordner: finden, eigenstaendig machen, Zip, Migration
├── mesh_utils.py               # STL pruefen/reparieren (nur Standardbibliothek)
├── build_env_scene.py          # kombiniert G1 + Umgebung (nutzt start.sh)
├── test_*.py                   # Tests (python3 test_<name>.py)
├── requirements.txt
├── meshes/                     # Mesh-BIBLIOTHEK (Quelle zum Einfuegen im Editor)
│   ├── sample_crate.stl        # Beispiel-STL zum Import-Testen
│   ├── sample_ramp.stl
│   ├── sample_bracket.step     # Beispiel-STEP zum CAD-Import-Testen
│   ├── cad/<name>/             # Zwischenablage fuer CAD-Importe im Editor (nicht im Git)
│   └── uploads/                # im Editor hochgeladene CAD-Dateien (nicht im Git)
├── scenes/                     # die UMGEBUNGEN - je eine ein Ordner
│   ├── environment_starter/
│   │   ├── umgebung.xml        # nur Objekte, Pfade relativ (meshes/...)
│   │   └── meshes/             # alles, was umgebung.xml braucht
│   └── dc_demonstrator/        # z.B. eine importierte CAD-Zelle
│       ├── umgebung.xml
│       └── meshes/dc_demonstrator/   # Einzelteile + cad_import.json
└── export/                     # Umgebungs-Zips (./launch.sh pack, nicht im Git)

../unitree_robots/g1/
    ├── scene_g1_playground.xml # statisches Beispiel: G1 + Objekte + STLs
    └── scene_env_<name>.xml     # auto-generiert bei Umgebungs-Auswahl (start.sh)
```

**Umgebung im G1-Sim laden:** `g1pilot/start.sh` (Sim) fragt „Welche
Umgebung laden?" und listet alle Umgebungen aus `scenes/` auf – der G1 wird
unveraendert hineingeladen (siehe Abschnitt 4).

> `meshes/` ist die **Bibliothek**, aus der man im Editor einfuegt; eine
> gespeicherte Umgebung haengt aber nie davon ab. Beim Speichern kopiert der
> Editor alles Benutzte in den Umgebungsordner.

---

## 1. Setup (einmalig)

Voraussetzung auf dem Host: **Python 3.10–3.12** (`mujoco-scene-editor`
unterstützt 3.13+ nicht). Ubuntu 22.04/24.04 passt; unter Ubuntu 26.04
(Python 3.14) vorher `curl -LsSf https://astral.sh/uv/install.sh | sh &&
~/.local/bin/uv python install 3.12`. `setup.sh` sucht sich automatisch
einen passenden Interpreter (oder `PYTHON=/pfad/zu/python3.12 ./setup.sh`).

Außerdem `python3-venv`. Ohne das Paket
bricht `python3 -m venv` mit `No module named 'ensurepip'` ab und
hinterlässt ein `.venv` ohne `pip`. `setup.sh` lädt die Pakete von PyPI,
braucht also Internet (unter WSL2 mit VPN siehe
[WSL2-Netzwerk](../../g1pilot/docs/01_installation.md#wsl2-netzwerk-vpn-uni-wlan-roboter)).

```bash
sudo apt install -y python3-venv
cd unitree_mujoco/scene_editor
rm -rf .venv          # nur bei altem/kaputtem venv, z.B. nach Verschieben des Repos
./setup.sh
.venv/bin/python -c "import trimesh, vhacdx; print('ok')"   # Kontrolle
```

Das legt ein eigenes `.venv/` an und installiert den Editor dort hinein
(die Systemumgebung wird nicht angefasst).

> **Der Editor laeuft bewusst auf dem Host, nicht im Docker-Stack.** Der
> Sim-Stack liest die fertigen Umgebungen nur (read-only gemountet).
> Dieses `setup.sh` ist also das einzige Setup, das der Editor braucht – einmal,
> danach nie wieder.
>
> Fehlt in einem **aelteren** `.venv` ein spaeter dazugekommenes Paket (z.B.
> `cadquery-ocp`/`coacd` fuer den CAD-Import oder `vhacdx`), installiert `launch.sh` es beim
> naechsten Start **automatisch nach** – `setup.sh` musst du dafuer nicht erneut
> aufrufen.

Warum ein eigenes venv? Der Editor zieht `GPUtil` mit, das sich mit dem
alten System-`setuptools` **nicht bauen** laesst. Im frischen venv mit
aktuellem `setuptools` klappt es. Ausserdem fehlt dem Editor-Paket die
Abhaengigkeit `yourdfpy` – die installiert `setup.sh` gleich mit.

### Setup unter Windows (oder ohne passendes Python): Docker

`setup.sh`/`launch.sh` brauchen ein Linux mit Python 3.10–3.12. Unter Windows
(Docker Desktop genuegt, kein WSL-Ubuntu noetig) laeuft beides in einem
Container: das Image fuehrt beim Bauen genau `setup.sh` aus, beim Start
`launch.sh`. Code, `scenes/` und `meshes/` werden live aus dem Repo gemountet
– gespeicherte Umgebungen landen also ganz normal hier im Ordner.

PowerShell, im Ordner `g1pilot/`:

```powershell
# einmalig bauen (~5 min, ~4.7 GB); nach Aenderungen an requirements.txt neu
docker compose --profile editor build scene-editor

# Menue von launch.sh (Editor dann im Browser: http://127.0.0.1:8080)
docker compose --profile editor run --rm --service-ports scene-editor

# direkt ein launch.sh-Kommando
docker compose --profile editor run --rm --service-ports scene-editor bash launch.sh edit kueche

# CAD-Datei ohne Editor als Umgebung importieren: Ordner als /import mounten
$env:SCENE_IMPORT_DIR = "C:\Users\<du>\Documents\CAD"
docker compose --profile editor run --rm scene-editor bash launch.sh import /import/zelle.stp
```

Unter Linux/WSL geht dasselbe mit `make editor` bzw.
`make editor ARGS="import /import/zelle.stp"`.

- Der Editor ist nur auf diesem Rechner erreichbar (Port auf `127.0.0.1`).
  Anderer Port: `$env:SCENE_EDITOR_PORT = "8081"`.
- Im Browser geht der Upload-Knopf fuer beliebige Windows-Dateien – `/import`
  braucht man nur fuer `launch.sh import`.
- Die Viewer-Kommandos (`view`, `with-g1`) brauchen ein Desktop-Fenster und
  gehen im Container nicht (`with-g1` baut und prueft die Szene trotzdem).
- **Git Bash** verbiegt Pfade wie `/import/...` zu Windows-Pfaden – dort
  vorher `export MSYS_NO_PATHCONV=1` setzen (PowerShell/WSL betrifft das nicht).

> **Erster Start braucht Internet:** Der Editor laedt beim allerersten Mal
> einmalig den Objaverse-Objektkatalog (Online-3D-Bibliothek) herunter und
> cached ihn. Danach laeuft er offline.

---

## 2. Starten – interaktives Menue

Einfach ohne Argument starten:

```bash
./launch.sh
```

Das listet **alle Umgebungen nummeriert** auf (aus dem zentralen Ordner
`scene_editor/scenes/` – dieselben, die auch beim G1-Start waehlbar sind):

```
=================== MuJoCo Scene Editor ===================
Umgebungen in scene_editor/scenes/
(dieselben, die auch beim G1-Start via g1pilot/start.sh waehlbar sind)
----------------------------------------------------------
    1) dc_demonstrator
    2) environment_starter
    n) neue leere Umgebung im Editor
    i) CAD-/Mesh-Datei (STEP, STL, ...) als Umgebung importieren
    u) Umgebung aus Zip einspielen
    q) beenden
----------------------------------------------------------
Auswahl (Zahl / n / i / u / q): 2

Gewaehlt: environment_starter
Aktion:
   e) im Editor bearbeiten
   v) allein im Viewer ansehen (ohne Roboter)
   g) mit dem G1 im Viewer ansehen
   p) als Zip packen (-> export/environment_starter.zip)
Auswahl [e/v/g/p] (Default e):
```

- Zahl = Umgebung waehlen, dann:
  - **e** – im Editor bearbeiten
  - **v** – allein im Viewer ansehen (ohne Roboter)
  - **g** – **mit dem G1** im Viewer ansehen (kombiniert automatisch, ohne
    Docker-Stack)
  - **p** – als Zip packen (zum Weitergeben, Abschnitt 5)
- **n** = mit einer leeren Umgebung neu anfangen.
- **i** = CAD-/Mesh-Datei direkt als neue Umgebung importieren.
- **u** = eine gezippte Umgebung einspielen.

Neue Umgebungen aus `scenes/` tauchen automatisch in der Liste auf – hier
**und** beim G1-Start.

## 3. Umgebung im Editor bauen & speichern

Der Editor startet einen lokalen Webserver und oeffnet den Browser
(**http://127.0.0.1:8080**). Dort kannst du:

- **Nur Objekte bauen** – Hindernisse / Greifobjekte. Boden, Licht, G1 und
  Weld kommen automatisch aus der Basis (nicht selbst anlegen).
- **Shapes platzieren** – Box, Kugel, Zylinder ... per Maus setzen/verschieben
- **Eigene STLs / STEP-CAD-Dateien importieren** – siehe eigener Abschnitt unten
  (der Knopf steckt im zugeklappten Ordner **„Add Assets from File"**).
- **Objekte benennen** – Ordner **„Objekt umbenennen"**: Objekt oben unter
  „Elements" waehlen, neuen Namen eintippen, **`Umbenennen`**. Praefix
  **`grasp_`** = greifbares Objekt, alles andere = Hindernis (siehe oben).
- **Speichern** – Ordner **„Umgebung speichern"** ganz oben. Dort steht **nur
  ein Feld: der Name** (z.B. `kueche`) – kein Pfad. Klick auf **`Speichern`**
  schreibt den Ordner `scenes/kueche/` (`umgebung.xml` + `meshes/`), und die
  Umgebung ist sofort im Menue **und** beim G1-Start waehlbar.

Beim Speichern passiert ausserdem automatisch:

- **Alle Dateien kommen in den Umgebungsordner.** Meshes aus der Bibliothek,
  CAD-Importe und Dateien von irgendwo auf der Platte werden nach
  `scenes/<name>/meshes/` kopiert und relativ referenziert (`meshes/...`).
  CAD-Importe bleiben als Unterordner samt `cad_import.json` zusammen (sonst
  kennt der Editor beim naechsten Oeffnen ihre Farben nicht mehr). Gleichnamige,
  aber verschiedene Dateien werden umbenannt statt ueberschrieben.
- **Aufraeumen:** Meshes, die nach dem Loeschen von Objekten keiner mehr
  benutzt, fliegen aus dem Ordner. Unter anderem Namen speichern = Kopie.
- **Kontrolle:** die gespeicherte Umgebung wird sofort testweise mit MuJoCo
  geladen. Klappt das nicht, sagt die Meldung im Editor warum (gespeichert wird
  trotzdem – deine Arbeit geht nie verloren).
- **Kein Beifang mehr:** frueher legte der Export zusaetzlich eine Datei
  `MuJoCo Model.xml` in `scenes/` ab, die dann als Geister-Umgebung in jeder
  Auswahlliste stand. Die wird jetzt weggeraeumt.

> „Umgebung speichern", „Objekt umbenennen", der Upload-Knopf und „Mesh
> skalieren" kommen aus `run_editor.py`; `launch.sh` startet den Editor immer
> darueber. Der eingebaute Export mit Pfad-Eingabe ist ausgeblendet.

### Eigene Meshes / CAD-Dateien in den Editor importieren

Es gibt zwei Wege – nimm den, der dir lieber ist:

**A) Datei-Dialog (beliebiger Ordner) – am einfachsten**

Ganz oben im Editor gibt es den Ordner **„Eigene Datei hochladen"** mit dem
Knopf **„Datei waehlen (STL/OBJ/STEP/...)"**:

1. Knopf klicken -> es oeffnet sich der **Datei-Dialog deines Systems**.
2. Datei aus **irgendeinem Ordner** waehlen: STL, OBJ, **STEP/STP, IGES/IGS,
   BREP**, PLY, GLB/GLTF, 3MF, OFF, DAE.
3. Fertig:
   * **STL/OBJ**, die MuJoCo direkt laden kann, kommen wie bisher als ein
     Mesh-Objekt in die Szene (ASCII-STL wird dabei binaer neu geschrieben).
   * **Alles andere** (CAD, fremde Formate, STL ueber 200000 Dreiecke) laeuft
     durch den CAD-Import (naechster Abschnitt) und kommt als **Gruppe aus
     Einzelteilen** in die Szene – mit Namen, Farben und Lage aus der Datei.
     Die ganze Gruppe unter „Elements" waehlen, um sie zu verschieben.

> Dieser Knopf wird von `run_editor.py` ergaenzt (der eingebaute Editor hat
> nur den Ordner-Scan unten). `launch.sh` startet den Editor immer darueber.

**B) Ordner-Scan (der eingebaute Weg)**

Der Editor kann auch einen Ordner nach Mesh-Dateien (STL, OBJ, PLY, GLB/GLTF,
USD) durchsuchen. Dieser Ordner ist fest auf `scene_editor/meshes/` vorbelegt:

1. Mesh nach `scene_editor/meshes/` kopieren.
2. Ordner **„Add Assets from File"** aufklappen (standardmaessig zugeklappt).
3. **„Scan assets"** klicken -> Meshes erscheinen im Dropdown
   (`sample_crate`/`sample_ramp` sind schon da).
4. Auswaehlen -> **„Add asset"**. Zu grosse Netze und Formate, die MuJoCo nicht
   kann (PLY/GLB), werden dabei automatisch ueber den CAD-Import zerlegt.

> Der eingebaute Editor-Default `~/temp/ArmarXObjects` existiert nicht – darum
> war die Liste vorher leer und der Import schien zu fehlen.

**CAD-Dateien aus `meshes/`** (STEP & Co. kennt der Ordner-Scan nicht): Ordner
**„CAD-Import"** aufklappen -> **„Liste aktualisieren"** -> Datei waehlen ->
**„Importieren & einfuegen"**.

### CAD-Import: STEP/IGES, ganze Anlagen

**MuJoCo kann kein CAD laden.** STEP/IGES beschreiben exakte Flaechen
(BRep/NURBS), MuJoCo braucht Dreiecksnetze (STL/OBJ/MSH). `cad_import.py`
**tesseliert** die Datei deshalb – und zwar **Bauteil fuer Bauteil**, so wie
sie im CAD aufgebaut ist:

| Was | Warum |
|---|---|
| **Je Bauteil (Volumenkoerper) ein Mesh** | MuJoCo kollidiert ueber die konvexe Huelle je Mesh. Eine ganze Zelle als ein Mesh waere ein einziger Klotz – und haette Millionen Dreiecke (MuJoCo-Grenze: 200000 je STL). |
| **Mehrfach verbaute Teile nur einmal gespeichert** | Profile, Schrauben usw. teilen sich eine Datei. |
| **Namen, Farben, Lage aus dem CAD** | Die Szene sieht aus wie im CAD; Teile heissen wie dort (`Zellenwand`, `Itemprofil_40x40x2120_3`). |
| **Einheit automatisch** | mm/inch/m aus der Datei -> Meter. Nur bei Meshes ohne Einheit ggf. Skalierung setzen. |
| **Kleine Teile nur Optik** | Schrauben, Knoepfe (Diagonale < 3 cm) kollidieren nicht – spart Rechenzeit. |
| **Konkave Teile konvex zerlegt** | Eine L-foermige Wand oder ein hohles Gehaeuse wuerde als konvexe Huelle die halbe Zelle fuellen. Solche Teile bekommen unsichtbare Kollisions-Stuecke `<teil>__k<n>` (Gruppe 3, braucht `coacd`). |
| **Offene/flache Teile** | Bleche/Schilder ohne Volumen bekommen `inertia="shell"` (sonst: „mesh volume is too small"). |
| **Platzierung `auto`** | Boden (bzw. Oberkante einer Bodenplatte) auf z=0; bei begehbaren Modellen steht der G1 (immer im Ursprung) auf einem freien Platz moeglichst mittig, kleine Objekte kommen 1 m vor den G1. Steht der G1 doch in einem Teil, sagt der Import das. |

Ergebnis auf der Kommandozeile: direkt eine fertige Umgebung
`scenes/<name>/` (`umgebung.xml` + Einzelteile samt Manifest `cad_import.json`
in `meshes/<name>/`). Im Editor landet der Import zuerst in der Zwischenablage
`meshes/cad/<name>/` und beim Speichern im Ordner der Umgebung. Wird dieselbe
Datei mit denselben Einstellungen noch einmal importiert, kommt das Ergebnis
sofort aus dem Cache. Ein erneuter Import ersetzt die Umgebung nur, solange sie
nicht im Editor weiterbearbeitet wurde – sonst bricht er mit Hinweis ab
(`--replace` ersetzt trotzdem, `--name` legt eine neue an).

Beispiel (NX-Export einer Roboterzelle, 43 MB STEP): 698 Teile, 315
verschiedene Meshes, 4 konkave Teile zerlegt – ca. 20 s Einlesen/Vernetzen plus
ca. 2 min fuer die konvexe Zerlegung, Laden in MuJoCo 0,3 s.

**Einstellungen** (im Editor Ordner **„CAD-Import"**, auf der Kommandozeile als
Optionen):

- **Genauigkeit** `coarse` / `normal` (Default) / `fine` – wie fein Rundungen
  vernetzt werden (mehr Dreiecke = schoener, aber langsamer).
- **Skalierung** `0` = automatisch, sonst fester Faktor (z.B. `0.001`).
- **Platzierung** `auto` (Default) / `floor` / `center` / `cad`.
- **Nur Optik unter (m)** – Grenze fuer „kleine Teile".

Auf der Kommandozeile (ohne Editor) – das Ergebnis ist sofort beim G1-Start
waehlbar:

```bash
./launch.sh import ~/Downloads/zelle.stp                  # -> scenes/zelle/
./launch.sh import zelle.stp --name demo --quality coarse
./launch.sh import teil.step --place cad                  # CAD-Koordinaten behalten
./launch.sh import --help                                 # alle Optionen
```

Technisch steckt dahinter **OpenCascade** (`cadquery-ocp`) fuer CAD,
**trimesh** fuer Mesh-Formate und **CoACD** (`coacd`) fuer die konvexe
Zerlegung – alles reine pip-Pakete, `setup.sh`/`launch.sh` installieren sie.
Fehlt `coacd`, kollidieren konkave Teile als gefuellter Block (der Import sagt,
welche). **JT, Parasolid (x_t), SolidWorks, Inventor** kann keine freie
Bibliothek lesen – im CAD als STEP (AP214/AP242) exportieren.

### STLs bewegen & skalieren

**Auswaehlen:** oben im Dropdown **„Elements"** das Mesh waehlen (oder im
3D-Fenster direkt draufklicken – „Allow mouse selection" ist an).

**Bewegen / Drehen:** ist das Mesh gewaehlt, erscheint der Zieh-Gizmo
(Pfeile/Ringe). Steuert die Checkbox **„Interactive translation"** (standard-
maessig **an**). Alternativ im **„Transform"**-Feld **Position (m)** / **Angles
(deg)** eintippen und **„Set Transform"** klicken. Sieht es aus, als bewege sich
nichts: erst ein Element auswaehlen und ggf. „Interactive translation" pruefen.

**Skalieren:** der eingebaute Editor kann Meshes **nicht** skalieren (nur
Box/Zylinder/Kugel-Masse). Dafuer gibt es den ergaenzten Ordner
**„Mesh skalieren"**:

1. Mesh oben unter „Elements" auswaehlen (das Feld zeigt dann seinen aktuellen
   Faktor).
2. **Faktor** eingeben (z.B. `2` = doppelt so gross; **CAD-STL in mm -> `0.001`**).
3. **„Auf gewaehltes Mesh anwenden"**.

> Upload-Button und „Mesh skalieren" ergaenzt `run_editor.py`; `launch.sh`
> startet den Editor immer darueber.

---

## 4. Umgebung im G1-Sim laden (der Hauptweg)

Der G1 bleibt **immer gleich** – du waehlst beim Start nur die Umgebung, und
der Roboter wird unveraendert hineingeladen. Beim G1-Start (`g1pilot/start.sh`,
Sim-Modus) kommt jetzt die Frage:

```
2d) Welche Umgebung laden? (G1 wird unveraendert hineingeladen)
   * 1) Standard — aktuelles Terrain (scene.xml)
     2) environment_starter
     3) kueche
     ...
```

Jede Umgebung aus `scene_editor/scenes/` taucht hier automatisch als Auswahl auf.
Waehlst du eine, passiert Folgendes automatisch:

1. `build_env_scene.py` baut auf dem Host eine kombinierte Szene
   `unitree_robots/g1/scene_env_<name>.xml` = **G1 + deine Umgebung**
   (mit passender Hand-Variante, Mesh-Pfade korrekt umgerechnet).
2. `start.sh` setzt `G1_ENV=<name>`; `config.py` laedt dann diese Szene.

Du musst also **nichts** von Hand zusammenkopieren – Umgebung im Editor bauen,
speichern, beim Start auswaehlen, fertig.

### Schnell ohne den G1-Stack ansehen

```bash
./launch.sh view-g1       # statisches Beispiel G1 + Objekte
```

Oder eine kombinierte Szene manuell erzeugen und im Viewer pruefen:

```bash
python3 build_env_scene.py --env kueche --inspire 0
.venv/bin/python -m mujoco.viewer --mjcf=../unitree_robots/g1/scene_env_kueche.xml
```

---

## Wie haengt das zusammen?

| Datei | Zweck | Inhalt |
|-------|-------|----------|
| `scenes/<name>/` | **Umgebungen – die baust du im Editor** | `umgebung.xml` (nur Objekte) + `meshes/` |
| `env_store.py` | kennt das Ordner-Format: finden, eigenstaendig machen, Zip, Migration | – |
| `build_env_scene.py` | baut **Basis** (G1+Licht+Boden+Weld) und mischt die Objekte ein | – |
| `scene_env_<name>.xml` (im g1-Ordner, auto-generiert) | **das laedt der Sim** | Basis + Objekte |

Der Generator schreibt die feste Basis (G1, Lichtquelle, Boden, Skybox und den
Weld `hold_base_weld`, der den G1 anfangs an `torso_link` festhaelt) selbst und
haengt nur die Objekte der Umgebung an. Eigene Boeden/Lichter/doppelte
Asset-Namen aus der Umgebung werden dabei automatisch aussortiert.

Warum ueberhaupt ein Generator und kein simples `<include>`? Das G1-Modell setzt
`<compiler meshdir="meshes">`, und dieses `meshdir` gilt in MuJoCo **global** –
auch fuer deine eigenen Meshes. Die kombinierte Szene muss im g1-Ordner liegen
(sonst fehlen die Roboter-Meshes), und eigene Mesh-Pfade muessen dazu passen.
`build_env_scene.py` erledigt das: kombinierte Szene in den g1-Ordner, Mesh-Pfade
der Umgebung relativ umgerechnet (gilt auch im read-only gemounteten
Docker-Container).

> `scene_g1_playground.xml` (im g1-Ordner) ist ein **statisches Beispiel** von
> Hand. Der eigentliche Weg fuer eigene Umgebungen ist der Generator oben.

---

## Eigene STLs – Kurzreferenz

In `scenes/<name>/umgebung.xml`, die STL liegt in `scenes/<name>/meshes/`:

```xml
<asset>
  <mesh name="tisch" file="meshes/tisch.stl" scale="0.001 0.001 0.001"/>
</asset>
<worldbody>
  <geom type="mesh" mesh="tisch" pos="1 0 0"/>
</worldbody>
```

- MuJoCo mag **binaeres** STL; `scale` beachten (CAD ist oft in mm).
- Fuer Kollision: Mesh moeglichst geschlossen/konvex. Reine Deko:
  `contype="0" conaffinity="0"`.

Mehr dazu in `meshes/README.md`.

---

## 5. Umgebungen weitergeben: Git, Zip, alte Umgebungen

**Git (Normalfall):** Umgebungsordner werden **samt Meshes eingecheckt** – nach
`git pull` ist eine Umgebung auf jedem Rechner sofort waehlbar. Git speichert
gleiche Dateien nur einmal, mehrere Umgebungen mit derselben Zelle kosten also
kaum Platz. Soll eine grosse Umgebung **nicht** ins Repo (z.B. eine Kunden-
Anlage), eine Zeile in `.gitignore`: `scenes/<name>/`.

**Zip (ohne Git, z.B. per USB/Cloud):**

```bash
./launch.sh pack dc_demonstrator             # -> export/dc_demonstrator.zip
./launch.sh pack dc_demonstrator ~/Desktop   # anderes Ziel (Ordner oder .zip)
./launch.sh unpack ~/Downloads/dc_demonstrator.zip          # -> scenes/dc_demonstrator/
./launch.sh unpack zelle.zip --name zelle_v2 --force        # umbenennen / ersetzen
```

Oder in der Start-GUI unter *Umgebungen bearbeiten*: **📦 Als Zip exportieren**
/ **📥 Zip einspielen**. Das Zip enthaelt genau den Ordner. Beim Einspielen
wird geprueft, dass alle Pfade relativ sind und im Ordner bleiben – ein kaputtes
oder fremdes Zip landet nie halb in `scenes/`.

**Pruefen:** `./launch.sh check` zeigt je Umgebung, ob sie eigenstaendig ist
(keine absoluten Pfade, nichts ausserhalb des Ordners, keine fehlende Datei).
Behebt sich durch einmal Oeffnen und Speichern im Editor.

**Alte Umgebungen** (flache `scenes/<name>.xml` mit Meshes irgendwo in
`meshes/`) wandelt `launch.sh` beim naechsten Start **automatisch** um
(`./launch.sh migrate` von Hand): Meshes werden in den neuen Ordner kopiert,
die Bibliothek bleibt unangetastet. Bis dahin funktionieren sie weiter.

---

## launch.sh – Uebersicht

```bash
./launch.sh                 # interaktives Menue (Umgebungen nummeriert waehlen)
./launch.sh list            # vorhandene Umgebungen auflisten
./launch.sh new             # leere Umgebung im Editor
./launch.sh edit [name]     # Umgebung im Editor (Default: environment_starter)
./launch.sh prompt "..."    # Umgebung per Text-Prompt generieren (braucht OPENAI_API_KEY)
./launch.sh view [name]     # Umgebung allein im MuJoCo-Viewer ansehen
./launch.sh with-g1 [name]  # Umgebung + G1 im MuJoCo-Viewer ansehen
./launch.sh view-g1         # statisches Beispiel scene_g1_playground.xml
./launch.sh import <datei>  # CAD/Mesh (STEP, IGES, STL, GLB, ...) -> scenes/<name>/
./launch.sh pack <name> [ziel]      # Umgebung als Zip (Default: export/<name>.zip)
./launch.sh unpack <zip> [--name N] [--force]   # Zip als Umgebung einspielen
./launch.sh check [name]    # Umgebung(en) eigenstaendig/vollstaendig?
./launch.sh migrate         # alte Umgebungen (scenes/<name>.xml) umwandeln
./launch.sh check-meshes    # alle STL (Bibliothek + Umgebungen) auf MuJoCo-Tauglichkeit
```

`[name]` darf `kueche`, `scenes/kueche` oder der Pfad zur `umgebung.xml` sein;
bei einem Tippfehler listet `launch.sh` die vorhandenen Umgebungen auf.
Dateipfade (`import`/`pack`/`unpack`) duerfen relativ zum aktuellen Ordner
sein. Anderer Port fuer den Editor: `SCENE_EDITOR_PORT=8081 ./launch.sh edit kueche`.

Nach Aenderungen an der Normalisierung: `python3 test_build_env_scene.py`,
an der STL-Pruefung: `python3 test_mesh_utils.py`, am Ordner-Format/Zip:
`python3 test_env_store.py` (alle drei nur Standardbibliothek),
am CAD-Import: `.venv/bin/python test_cad_import.py` (im venv laufen alle Tests,
mit blankem `python3` werden die Teile ohne numpy/trimesh/OpenCascade
uebersprungen).

## Bekannte Stolpersteine

- **`stl_decoder: number of faces should be between 1 and 200000 ...
  perhaps this is an ASCII file?`** – MuJoCo laedt nur **binaeres** STL, kein
  ASCII-STL (haeufig bei Downloads/Exporten aus Blender, Sketchfab,
  Objaverse). Wird automatisch behoben: beim Speichern im Editor prueft
  `run_editor.py` alle verwendeten STLs und schreibt ASCII-Dateien binaer neu
  (mit `trimesh`); `build_env_scene.py` macht dieselbe Reparatur beim
  Kombinieren nochmal als Sicherheitsnetz (z.B. fuer Umgebungen, die nicht
  ueber den Editor entstanden sind). Fehlt `trimesh` (nur beim Aufruf mit
  blankem System-`python3` moeglich) oder laesst sich die Datei nicht
  reparieren, wird nur DAS betroffene Objekt weggelassen (mit Warnung) statt
  die ganze Umgebung zu verwerfen – von Hand reparieren: Datei in
  Blender/MeshLab als binaeres STL neu exportieren.
- **`Speichern fehlgeschlagen: AttributeError: 'NoneType' object has no
  attribute 'joints'`** – Bug im Fremdpaket: neue Objekte bekommen als
  Eltern-Pfad das, was gerade oben unter „Elements" ausgewaehlt ist. War dabei
  zufaellig ein normales Objekt (Box/Mesh/...) statt eine echte Gruppe
  ausgewaehlt, haengt der Editor das neue Objekt intern darunter – der
  Exporter kennt aber nur Gruppen als Eltern und stuerzt sonst ab.
  `run_editor.py` faengt das seit kurzem ab: verwaiste Objekte werden vor dem
  Speichern automatisch auf die oberste Ebene gehoben (Hinweis erscheint in
  der Speichern-Meldung) – nichts geht verloren. Vorbeugen: vor „Create Box"
  o.ae. in „Elements" erst **„— none —"** waehlen, wenn du keine Gruppe
  meinst.
- **`ModuleNotFoundError: yourdfpy`** – `setup.sh` erneut laufen lassen
  (installiert es); oder `.venv/bin/pip install yourdfpy`.
- **Build-Fehler bei `GPUtil` / `install_layout`** – passiert nur bei
  Installation in die Systemumgebung. Immer das venv aus `setup.sh` nutzen.
- **CAD-Import inaktiv / „OpenCascade fehlt"** – `launch.sh` holt die Pakete
  beim naechsten Start automatisch nach (braucht einmalig Internet). Steht es
  danach immer noch da: **Editor neu starten** (der Ordner „CAD-Import" wird nur
  beim Start aufgebaut). Von Hand: `.venv/bin/pip install cadquery-ocp coacd`;
  pruefen mit `.venv/bin/python cad_import.py --check` (Exit 0 = alles da).
- **`stl_decoder: number of faces should be between 1 and 200000` /
  „perhaps this is an ASCII file?"** – MuJoCos zwei STL-Grenzen: es laedt **nur
  binaeres** STL und **hoechstens 200000 Dreiecke** je Datei. Beides faengt der
  Editor selbst ab:
  * ASCII-STL wird beim Import automatisch binaer neu geschrieben.
  * Zu grosse Netze (auch per „Add asset") werden ueber den CAD-Import in
    mehrere Meshes geteilt statt abgelehnt; CAD-Baugruppen werden ohnehin
    Bauteil fuer Bauteil vernetzt.

  Bestand pruefen: `./launch.sh check-meshes`.
- **`mesh volume is too small ... Try setting inertia to shell`** – ein
  offenes/flaches Mesh (Blech, Schild). Der CAD-Import setzt dafuer automatisch
  `inertia="shell"`, auch beim Speichern im Editor und beim Kombinieren. Bei
  eigenen Meshes: `<mesh ... inertia="shell"/>` von Hand setzen.
- **CAD-Teil ist 1000x zu gross/klein** – bei STEP/IGES kommt die Einheit aus
  der Datei; falsch ist sie praktisch nur bei Meshes (STL/OBJ haben keine
  Einheit) oder BREP (mm angenommen). Im Ordner „CAD-Import" die Skalierung
  setzen (z.B. `0.001`) und neu importieren, oder `--scale` auf der
  Kommandozeile.
- **Der G1 steht beim Start in einem Teil der Anlage** – der Import meldet das
  („ACHTUNG: Der G1 startet ... IN ..."). Mit Platzierung `auto` importieren
  (sucht einen freien Platz) oder die Gruppe im Editor verschieben; in der
  erzeugten XML ist es das `pos` des einen `<body>` direkt unter `<worldbody>`.
- **Roboter kommt nicht in die Zelle / alles blockiert** – ein konkaves Teil
  (L-Wand, hohles Gehaeuse) kollidiert als gefuellter Block, weil `coacd` fehlt.
  Der Import nennt die Teile; `.venv/bin/pip install coacd` und neu importieren.
- **Absturz beim Start mit `403 Forbidden` / objaverse** – kein/gesperrtes
  Internet beim ersten Start. Der Objaverse-Katalog wird beim ersten Lauf
  einmalig heruntergeladen; mit Internet einmal starten, danach offline ok.
- **`Port 8080 ist schon belegt`** – es laeuft noch ein Editor. In der GUI:
  *Laufende Prozesse* -> *Stoppen*. Oder anderen Port nehmen:
  `SCENE_EDITOR_PORT=8081 ./launch.sh edit kueche`.
- **Umgebung erscheint nicht in der Auswahl** – der Ordnername darf nur
  `A-Z a-z 0-9 _ -` enthalten (er wird als `G1_ENV` weitergereicht), darin muss
  eine `umgebung.xml` (ein `<mujoco>`-XML) liegen. Die GUI zeigt aussortierte
  Eintraege unter *Umgebungen bearbeiten* mit Begruendung an.
- **Umgebung laedt auf einem anderen Rechner nicht / Meshes fehlen** – sie
  verweist noch auf Dateien ausserhalb ihres Ordners. `./launch.sh check <name>`
  zeigt welche; einmal im Editor oeffnen und speichern holt sie hinein.
- **`Umgebung ... laesst sich nicht laden`** beim Start – die kombinierte Szene
  kompiliert nicht (meist ein fehlendes Mesh oder ein doppelter Name). Genaue
  Meldung: *Umgebungen bearbeiten* -> **„Auf Ladbarkeit pruefen"** bzw.
  `python3 build_env_scene.py --env <name>`.
- **Objekte fallen beim Start um** – das war der alte Zustand (der Editor
  exportiert alles mit freiem Gelenk). Heute wird normalisiert: beweglich ist
  nur noch, was `grasp_` im Namen hat.
- **Editor mangelt beim Import einer Roboter-Szene** – bekannt: der Editor
  verwirft beim XML-Import manche Tags (Joints/Aktuatoren ausserhalb der
  Roboterbeschreibung, Reibung). Deshalb im Editor nur die roboterfreie
  Umgebung bauen, nicht die volle G1-Szene.
- **Greif-Objekte fliegen beim Start weg / staendige `[scene-reset]`-Meldungen
  im Log** – `build_env_scene.py` braucht `trimesh` + `vhacdx` (jetzt in
  `requirements.txt`, `./setup.sh` installiert sie mit), um Mesh-Hindernisse
  wie den Arbeitsplatz in mehrere konvexe Teil-Huellen zu zerlegen. Fehlen
  die Pakete (z.B. `.venv` nach einem frischen `docker pull`/Checkout nicht
  neu aufgesetzt), faellt der Generator **still** auf die EINE konvexe Huelle
  des ganzen Meshes zurueck -- die "fuellt" bei einem Tisch/Regal den
  offenen Raum auf, abgelegte Greif-Objekte starten dann IN der Huelle und
  werden im ersten Sim-Schritt weggeschossen. Pruefen: `.venv/bin/python3 -c
  "import trimesh, vhacdx"` sollte ohne Fehler laufen; sonst `./setup.sh`
  erneut ausfuehren (braucht genug freien Diskspeicher!) und danach die
  betroffene(n) Szene(n) neu bauen (`.venv/bin/python build_env_scene.py
  --env <name>`, oder einfach `g1pilot/start.sh` neu starten). Meshes aus dem
  **CAD-Import** betrifft das nicht – die zerlegt `cad_import.py` schon beim
  Import selbst (CoACD).
