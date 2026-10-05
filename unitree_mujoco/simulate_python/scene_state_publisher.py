# -*- coding: utf-8 -*-
"""
scene_state_publisher.py — sendet die Umgebungs-Objekte (Hindernisse + greif-
bare Objekte) periodisch per UDP an den ROS-Container, damit sie dort in RViz
angezeigt und von Nav/IK als Kollisionsgeometrie genutzt werden koennen (siehe
g1pilot/g1pilot/navigation/scene_bridge.py, der Empfaenger).

Warum UDP statt ROS-Topic? Wie bei push_listener.py/grasp_box.py: der MuJoCo-
Container hat KEIN ROS. Beide Container laufen mit network_mode: host ->
Loopback verbindet sie ohne DDS-IDL/ROS hier. Anders als bei den bestehenden
Listenern ist die Richtung hier umgekehrt (MuJoCo -> ROS statt ROS -> MuJoCo)
und die Payload reicher (volle Objektliste statt eines Toggle-Bits) -> JSON
statt eines einzelnen Bytes/Zahl.

Protokoll v2 (JSON, ein Datagramm = ein Teilstueck):
    {"v": 2, "kind": "obstacles"|"grasp", "gen": <int>, "seq": <int>,
     "part": i, "parts": n, "items": [...]}
Ein Datagramm fasst hoechstens ~64 KB - eine CAD-Umgebung mit Hunderten
Teilen ist deutlich groesser (700 Teile ~ 220 KB). Darum wird jede Liste in
Stuecke von hoechstens MAX_DATAGRAM Bytes geteilt; der Empfaenger setzt eine
Liste erst zusammen, wenn alle `parts` Stuecke derselben `seq` da sind
(g1pilot/g1pilot/navigation/scene_protocol.py). `gen` wechselt bei jedem
Sim-Start, damit Reste eines alten Laufs nicht mit dem neuen gemischt werden.

  * Hindernisse haben eine FESTE Pose (aus der Szenen-XML, einmalig geparst)
    und werden nur alle OBSTACLE_PERIOD Sekunden erneut gesendet (fuer einen
    spaet gestarteten Empfaenger / verlorene Pakete).
  * Greifbare Objekte werden JEDEN Publish-Tick mit ihrer LIVE-Pose aus
    mj_data gesendet (freie Koerper, siehe build_env_scene.py).

Jedes Objekt: {name, class, type, pos[3], quat[4 wxyz], rgba[4], size,
mesh (Pfad unter scene_editor/, z.B. scenes/<name>/meshes/teil.stl), aabb_half}.
"""
import json
import os
import socket
import time

import scene_objects

#: Nutzlast je Datagramm (Bytes). Deutlich unter dem UDP-Maximum von 65507,
#: damit auch mit etwas Protokoll-Overhead nichts abgeschnitten wird.
MAX_DATAGRAM = 48000

#: Wie oft die (statischen) Hindernisse erneut gesendet werden (s).
OBSTACLE_PERIOD = 1.0


def encode_chunks(kind, gen, seq, items, max_bytes=MAX_DATAGRAM):
    """Liste in JSON-Datagramme <= max_bytes teilen. -> [bytes]

    Ein einzelnes Objekt, das allein schon zu gross ist, wird trotzdem
    gesendet (eigenes Datagramm) - besser als es still zu verlieren.
    """
    encoded = [json.dumps(it, separators=(",", ":")) for it in items]
    groups, cur, size = [], [], 0
    overhead = 160
    for e in encoded:
        if cur and size + len(e) + 1 + overhead > max_bytes:
            groups.append(cur)
            cur, size = [], 0
        cur.append(e)
        size += len(e) + 1
    groups.append(cur)                      # auch leer: "Liste ist leer" melden
    n = len(groups)
    return [
        ('{"v":2,"kind":%s,"gen":%d,"seq":%d,"part":%d,"parts":%d,"items":[%s]}'
         % (json.dumps(kind), gen, seq, i, n, ",".join(g))).encode("utf-8")
        for i, g in enumerate(groups)
    ]


class SceneStatePublisher:
    def __init__(self, mj_model, config):
        self.enabled = bool(getattr(config, "SCENE_ENABLE", True))
        self.port = int(getattr(config, "SCENE_UDP_PORT", 47902))
        self.host = str(getattr(config, "SCENE_UDP_HOST", "127.0.0.1"))
        self.hz = float(getattr(config, "SCENE_PUBLISH_HZ", 10.0))
        self._period = 1.0 / self.hz if self.hz > 0 else 0.1
        self._next_t = 0.0
        self._next_obstacles_t = 0.0
        self._sock = None
        self._gen = int(time.time() * 1000) & 0x7FFFFFFF
        self._seq = 0

        self.obstacles = []
        self.grasp = []   # je Eintrag: dict + "body_id"
        self._obstacle_chunks = []

        if not self.enabled:
            print("[scene] deaktiviert (SCENE_ENABLE=0).")
            return

        scene_path = os.path.abspath(config.ROBOT_SCENE)
        try:
            specs = scene_objects.parse_scene_objects(scene_path)
        except Exception as e:
            print(f"[scene] WARN: Szene konnte nicht geparst werden ({e}) -> "
                  f"keine Szenen-Objekte in RViz/Nav/IK.")
            specs = []

        for spec in specs:
            if spec["class"] == "grasp":
                try:
                    body_id = mj_model.body(spec["name"]).id
                except Exception:
                    print(f"[scene] WARN: Greif-Objekt '{spec['name']}' nicht im "
                          f"kompilierten Modell gefunden -> uebersprungen "
                          f"(braucht <freejoint/>, siehe build_env_scene.py).")
                    continue
                spec["body_id"] = body_id
                self.grasp.append(spec)
            else:
                self.obstacles.append(spec)

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as e:
            print(f"[scene] WARN: UDP-Socket konnte nicht angelegt werden: {e}")
            self.enabled = False
            return

        # Hindernisse sind statisch: einmal kodieren, danach nur noch senden.
        self._seq += 1
        self._obstacle_chunks = encode_chunks(
            "obstacles", self._gen, self._seq, self._obstacle_payload())
        print(f"[scene] Szenen-Publisher aktiv: {len(self.obstacles)} Hindernis(se), "
              f"{len(self.grasp)} Greif-Objekt(e) -> {self.host}:{self.port} @ "
              f"{self.hz:.0f} Hz ({len(self._obstacle_chunks)} Paket(e) Hindernisse, "
              f"scene_bridge, ROS-Seite).")

    @staticmethod
    def _item(o, cls, pos, quat):
        return {
            "name": o["name"], "class": cls, "type": o["type"],
            "pos": pos, "quat": quat, "rgba": o["rgba"], "size": o["size"],
            "mesh": o.get("mesh_resource") or o.get("mesh_basename"),
            "aabb_half": o["aabb_half"],
        }

    def _obstacle_payload(self):
        # "display" (nur Anzeige, z.B. Optik-Mesh eines in Teile zerlegten
        # Arbeitsplatzes) MUSS durchgereicht werden -- sonst sieht die IK
        # wieder die Riesen-Box um das ganze Mesh (siehe scene_objects.py).
        return [self._item(o, o.get("class", "obstacle"), o["pos"], o["quat"])
                for o in self.obstacles]

    def _grasp_payload(self, mj_data):
        out = []
        for g in self.grasp:
            bid = g["body_id"]
            pos = mj_data.xpos[bid]
            quat = mj_data.xquat[bid]   # MuJoCo: (w, x, y, z)
            out.append(self._item(
                g, "grasp",
                [float(pos[0]), float(pos[1]), float(pos[2])],
                [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]))
        return out

    def _send(self, datagrams):
        for data in datagrams:
            try:
                self._sock.sendto(data, (self.host, self.port))
            except OSError:
                pass   # Empfaenger (scene_bridge) laeuft evtl. gerade nicht -- kein Problem.

    def maybe_publish(self, mj_data):
        """Pro Sim-Schritt aufrufen (billig: tut ausserhalb des Zeitrasters
        nichts). Sendet nie eine Exception in die Physik-Schleife weiter."""
        if not self.enabled or self._sock is None:
            return
        now = time.perf_counter()
        if now < self._next_t:
            return
        self._next_t = now + self._period
        if now >= self._next_obstacles_t:
            self._next_obstacles_t = now + OBSTACLE_PERIOD
            self._send(self._obstacle_chunks)
        try:
            self._seq += 1
            chunks = encode_chunks("grasp", self._gen, self._seq,
                                   self._grasp_payload(mj_data))
        except Exception as e:
            print(f"[scene] WARN: Snapshot konnte nicht kodiert werden: {e}")
            return
        self._send(chunks)
