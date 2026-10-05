# Navigation — Technik

Richtet sich an: Entwickler, die den Navigations-Stack ändern oder erweitern
wollen. Für die Bedienung siehe
[50_navigation_anleitung.md](50_navigation_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/navigation/scene_bridge.py` | Umgebungsobjekte (Sim, per UDP) → `/scene_markers` |
| `g1pilot/navigation/scene_protocol.py` | setzt die in UDP-Pakete geteilten Szenen-Snapshots zusammen (ohne ROS testbar) |
| `g1pilot/navigation/create_map.py` | `/scene_markers` → 2D-Belegungskarte `/map` |
| `g1pilot/navigation/dijkstra_planner.py` | globaler Pfadplaner |
| `g1pilot/navigation/nav2point.py` | Pure-Pursuit-Waypoint-Follower |
| `g1pilot/navigation/sim_localization.py` | Sim-Ersatz für MOLA (Pose aus Ground-Truth) |
| `g1pilot/navigation/fix_mola_odometry.py` (`mola_fixed`) | Orientierungs-Fix der MOLA-Odometrie (nur Real) |
| `g1pilot/teleoperation/joy_mux.py`, `navigation/joy_to_cmdvel.py` | Kopplung an die Loco-Schicht |

## Datenfluss

```
   Ziel  /g1pilot/goal (PoseStamped, map)
     │
     ▼
 ┌───────────────────┐      /map (OccupancyGrid)        ┌──────────────────────┐
 │ dijkstra_planner  │◄──── create_map ──────────────────│  Pose                │
 │ Grid-Dijkstra +   │                                   │  /lidar_odometry/    │
 │ Inflation + LOS-  │◄──────────────────────────────────│  pose_fixed          │
 │ Shortcut + Glättung│                                  │                      │
 └─────────┬─────────┘                                   │ sim_localization      │
           │ /g1pilot/path                               │  (rt/sportmodestate)  │
           ▼                                              │ real: MOLA + fix     │
 ┌───────────────────┐                                    └──────────┬───────────┘
 │ nav2point         │◄────────────── Pose ──────────────────────────┘
 │ Pure-Pursuit-     │
 │ Waypoint-Follower │────► /g1pilot/auto_joy  (virtueller Joystick, Deadman-Btn 8)
 └───────────────────┘                    │
                                          ▼
                         ┌────────────────────────────┐   /g1pilot/auto_enable
                         │ joy_mux (Auto/Manuell-Mux)  │◄── (scharfschalten)
                         └──────────────┬──────────────┘
                                        │ /g1pilot/joy
                       ┌────────────────┴─────────────────┐
                       ▼ Sim                               ▼ Real
             ┌───────────────────┐                ┌──────────────────┐
             │ joy_to_cmdvel     │                │ loco_client      │
             │ Joy → loco_cmd_vel│                │ Move()           │
             └─────────┬─────────┘                └──────────────────┘
                       ▼
             ┌───────────────────┐
             │ loco_sim (Policy) │
             └───────────────────┘
```

## Node für Node

### `scene_bridge` (nur Sim)

Empfängt die Umgebungsobjekte (Hindernisse + greifbare Objekte) per UDP vom
MuJoCo-Container (`scene_state_publisher.py` in `unitree_mujoco`, Sender) und
veröffentlicht sie als `visualization_msgs/MarkerArray` auf
`/scene_markers` — QoS `TRANSIENT_LOCAL`, damit ein später gestarteter
Abnehmer sofort den letzten Stand bekommt. Grund für UDP statt ROS-Topic:
der MuJoCo-Container hat kein ROS; beide Container laufen mit
`network_mode: host`, Loopback verbindet sie ohne DDS/ROS.

Ein UDP-Datagramm fasst höchstens ~64 KB — eine CAD-Umgebung mit Hunderten
Teilen ist deutlich größer (700 Teile ≈ 220 KB). Der Sender teilt jede Liste
deshalb in Pakete ≤ 48 KB (Protokoll v2: `kind`, `gen`, `seq`, `part`,
`parts`), `scene_protocol.SnapshotAssembler` setzt sie wieder zusammen und
übernimmt eine Liste erst, wenn **alle** Teile da sind. Statische Hindernisse
kommen 1×/s, greifbare Objekte mit Live-Pose 10×/s. `scene_bridge`
veröffentlicht nur bei Änderungen (plus Keepalive `keepalive_s`, Default 2 s),
damit IK und Karte nicht 10×/s Hunderte Marker umrechnen. Mesh-Marker
verweisen auf den Pfad unter `scene_editor/meshes/` (z. B.
`file:///scene_editor/scenes/<name>/meshes/<teil>.stl`; der ROS-Container mountet dafür `unitree_mujoco/scene_editor` read-only).

`/scene_markers` ist das **eine geteilte Weltmodell**:

- RViz zeigt es direkt an.
- `create_map` rastert daraus die 2D-Karte.
- `arm_controller`/`ik_solver` speisen daraus die Umgebungskollision
  (siehe [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md)).

### `create_map`

Rastert die aus `/scene_markers` gemeldeten Objekte (Hindernisse **und**
Greif-Objekte — ein greifbares Objekt am Boden ist bis zum Greifen trotzdem
ein Hindernis für die laufende Basis) in eine `OccupancyGrid` (`/map`,
Default 100×100 Zellen à 0,1 m, zentriert im Ursprung). Auf echter Hardware
ohne entsprechendes Setup bleibt die Karte leer (kein `/scene_markers`-Sender).

Gestempelt wird nur, was in den Höhenbereich des Roboters ragt
(`obstacle_min_z` 0,05 m … `obstacle_max_z` 1,8 m): eine Bodenplatte, auf der
der G1 läuft, oder eine Kabelpritsche über Kopf sperren sonst die ganze Fläche
darunter (typisch bei importierten CAD-Zellen).

### `dijkstra_planner`

Klassischer Grid-Dijkstra mit:

- **Inflation** (`inflation_radius_m`, Default 0.40 m) — Sicherheitsabstand
  um belegte Zellen.
- **Turn-Cost** (`turn_cost_gain`) — bevorzugt gerade Wege gegenüber
  Zickzack.
- **Line-of-Sight-Shortcut** (`shortcut_path`) — entfernt unnötige
  Zwischenpunkte, wenn die direkte Sichtlinie frei ist.
- **Catmull-Rom-Glättung** (`_catmull_rom_centripetal`) — glättet den
  Rasterpfad zu einer weichen Kurve.

Ohne gültige Karte oder bei Start/Ziel außerhalb bzw. in einer belegten
Zelle fällt der Planer auf eine direkte, geglättete Linie zurück
(`line_points`). Ausgabe: `/g1pilot/path`.

### `nav2point`

Pure-Pursuit-Follower: zielt auf einen Punkt in fixem Vorlauf-Abstand
(`lookahead`) auf dem Pfad, regelt Position (`pos_kp`) und Ausrichtung
(`yaw_kp`) proportional, transformiert das Ergebnis ins Roboter-Koordinatensystem
und kodiert es als `sensor_msgs/Joy` mit gedrücktem Deadman-Button (Index 8)
auf `/g1pilot/auto_joy` — als ob ein Mensch den Joystick hielte.

Endphase (`final_align`): erreicht der Roboter die Zieltoleranz
(`goal_tolerance`), schaltet der Node in eine Ausrichtungsphase
(`aligning=True`) um: keine Translation mehr (verhindert Umkreisen), nur
noch Drehen auf den mitgelieferten Ziel-Yaw (`yaw_tol_deg` Toleranz). Ab
`yaw_hold_dist` vor dem Ziel wird das kontinuierliche Nachdrehen zum
aktuellen Wegpunkt abgeschaltet (Orbit-Fix) — sonst würde der Roboter
knapp vor dem Ziel um den (dann instabilen) Punkt kreisen.

### `sim_localization` (nur Sim) / MOLA + `mola_fixed` (nur Real)

`sim_localization` liest die Basis-Position/-Geschwindigkeit aus
`rt/sportmodestate` (MuJoCo-Ground-Truth) und die Orientierung aus der
IMU in `rt/lowstate`, und veröffentlicht daraus exakt die Topics, die der
Rest des Stacks erwartet: `/lidar_odometry/pose_fixed`
(`nav_msgs/Odometry`, Frame `map`) und die dynamische TF
`odom_unitree → base_link`. Auf echter Hardware übernimmt MOLA
(Livox-SLAM) dieselbe Rolle; `mola_fixed` transformiert deren Odometrie in
das vom Stack erwartete Frame.

### `joy_mux` / `joy_to_cmdvel`

Siehe [41_teleoperation_technik.md](41_teleoperation_technik.md). Auf real
liest `loco_client` `/g1pilot/joy` direkt; in der Simulation liest
`loco_sim` keinen `Joy`, daher übersetzt `joy_to_cmdvel` das (bereits durch
`joy_mux`/`auto_enable` gegatete) `/g1pilot/joy` in `/g1pilot/loco_cmd_vel`
(mit Watchdog: bleibt der Strom aus, wird einmalig 0 gesendet).

## Warum der Umweg über einen „Fake-Joystick"?

Damit die Autonomie exakt denselben, erprobten Pfad nimmt wie die manuelle
Teleoperation — inklusive Deadman, Geschwindigkeitslimits und Notaus. Es
gibt keinen zweiten, ungetesteten Steuerweg zum Roboter.

## Sim vs. Real — Bausteintabelle

| Baustein | Sim | Real |
|---|---|---|
| Lokalisierung | `sim_localization` | MOLA + `mola_fixed` |
| Planer | `dijkstra_planner` | identisch |
| Follower | `nav2point` | identisch |
| Joy-Mux | `joy_mux` | identisch |
| Loco-Anbindung | `joy_to_cmdvel` → `loco_sim` | Joy direkt → `loco_client` |
| Karte | `create_map` aus `/scene_markers` | `create_map`, i. d. R. leer ohne eigenes Setup |

## Bekannte Einschränkungen

- Keine Live-Perzeption: die Karte spiegelt nur die beim Sim-Start geladene
  Umgebung, keine spontanen Hindernisse.
- Auf echter Hardware ist ein Livox MID360 + das große Docker-Image
  Voraussetzung für Navigation (`G1_ENABLE_LIDAR=1`).
