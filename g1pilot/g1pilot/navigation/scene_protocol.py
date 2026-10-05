#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scene_protocol — setzt die UDP-Szenen-Snapshots des MuJoCo-Containers wieder
zusammen (Gegenstueck zu unitree_mujoco/simulate_python/scene_state_publisher.py).

Bewusst ohne ROS-Abhaengigkeit, damit es ohne ROS-Umgebung testbar ist
(test/test_scene_protocol.py).

Protokoll v2: jede Liste ("obstacles" = statische Hindernisse, "grasp" =
greifbare Objekte mit Live-Pose) wird in Datagramme <= ~48 KB geteilt:
    {"v": 2, "kind": ..., "gen": ..., "seq": ..., "part": i, "parts": n, "items": [...]}
Eine Liste gilt erst, wenn ALLE Teile derselben (gen, seq) angekommen sind -
sonst wuerde eine halb empfangene Szene kurz Objekte "verschwinden" lassen.
Ein neues `gen` (Sim neu gestartet) verwirft alles Alte.

Protokoll v1 (ein Datagramm {"obstacles": [...], "grasp": [...]}) wird
weiter verstanden.
"""
import json
import time

KINDS = ("obstacles", "grasp")


class SnapshotAssembler:
    """Datagramme rein, vollstaendige Objektlisten raus."""

    #: Unvollstaendige Teil-Listen nach dieser Zeit verwerfen (s).
    STALE_AFTER = 5.0

    def __init__(self):
        self.lists = {k: None for k in KINDS}   # zuletzt VOLLSTAENDIGE Listen
        self.gen = None
        self._partial = {}                      # (kind, seq) -> {part: items, ...}
        self._version = 0                       # zaehlt jede Aenderung von lists

    @property
    def version(self) -> int:
        return self._version

    def ready(self) -> bool:
        """Hindernisse vollstaendig da (Greif-Liste darf noch fehlen)?"""
        return self.lists["obstacles"] is not None

    def objects(self) -> list:
        return list(self.lists["obstacles"] or []) + list(self.lists["grasp"] or [])

    def feed(self, data: bytes, now: float = None) -> bool:
        """Ein Datagramm verarbeiten. -> True, wenn sich eine Liste geaendert hat."""
        now = time.monotonic() if now is None else now
        try:
            msg = json.loads(data.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return False
        if not isinstance(msg, dict):
            return False
        if msg.get("v") != 2:
            return self._feed_v1(msg)
        try:
            kind, gen, seq = msg["kind"], int(msg["gen"]), int(msg["seq"])
            part, parts = int(msg["part"]), int(msg["parts"])
            items = list(msg["items"])
        except (KeyError, TypeError, ValueError):
            return False
        if kind not in KINDS or not (0 <= part < parts):
            return False
        if gen != self.gen:                      # Sim neu gestartet
            self.gen = gen
            self._partial.clear()
            self.lists = {k: None for k in KINDS}
        self._drop_stale(now)
        buf = self._partial.setdefault((kind, seq), {"t": now, "parts": parts, "got": {}})
        buf["got"][part] = items
        if len(buf["got"]) < buf["parts"]:
            return False
        del self._partial[(kind, seq)]
        # Aeltere, nie fertig gewordene Stuecke derselben Liste sind jetzt wertlos.
        for key in [k for k in self._partial if k[0] == kind and k[1] < seq]:
            del self._partial[key]
        full = [it for i in range(buf["parts"]) for it in buf["got"][i]]
        if full == self.lists[kind]:
            return False
        self.lists[kind] = full
        self._version += 1
        return True

    def _feed_v1(self, msg: dict) -> bool:
        if "obstacles" not in msg and "grasp" not in msg:
            return False
        changed = False
        for kind in KINDS:
            items = list(msg.get(kind, []))
            if items != self.lists[kind]:
                self.lists[kind] = items
                changed = True
        if changed:
            self._version += 1
        return changed

    def _drop_stale(self, now: float) -> None:
        for key in [k for k, b in self._partial.items() if now - b["t"] > self.STALE_AFTER]:
            del self._partial[key]
