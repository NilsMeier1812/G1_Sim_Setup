#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Szenen-UDP-Protokoll: Sender (MuJoCo-Container) -> Empfaenger (scene_bridge).

Grosse CAD-Umgebungen passen nicht in ein UDP-Datagramm (max. ~64 KB) - der
Sender teilt, der Empfaenger setzt zusammen. Laeuft ohne ROS (stdlib + pytest).
"""
import importlib.util
import json
import random
from pathlib import Path

import pytest

from g1pilot.navigation.scene_protocol import SnapshotAssembler

_SENDER = (Path(__file__).resolve().parents[2] / "unitree_mujoco" / "simulate_python"
           / "scene_state_publisher.py")


def _load_sender():
    if not _SENDER.is_file():
        pytest.skip("unitree_mujoco/simulate_python nicht im Baum")
    import sys
    sys.path.insert(0, str(_SENDER.parent))          # fuer 'import scene_objects'
    spec = importlib.util.spec_from_file_location("scene_state_publisher", _SENDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _objects(n):
    return [{"name": f"teil_{i}", "class": "obstacle", "type": "mesh",
             "pos": [i * 0.1, 0.0, 0.5], "quat": [1.0, 0.0, 0.0, 0.0],
             "rgba": [0.5, 0.5, 0.5, 1.0], "size": [1.0, 1.0, 1.0],
             "mesh": f"cad/zelle/teil_{i}.stl", "aabb_half": [0.1, 0.2, 0.3]}
            for i in range(n)]


def test_grosse_szene_wird_geteilt_und_wieder_zusammengesetzt():
    sender = _load_sender()
    objs = _objects(700)
    chunks = sender.encode_chunks("obstacles", gen=7, seq=1, items=objs)
    assert len(chunks) > 1
    assert all(len(c) <= sender.MAX_DATAGRAM for c in chunks)

    asm = SnapshotAssembler()
    random.Random(0).shuffle(chunks)                   # UDP: Reihenfolge egal
    results = [asm.feed(c, now=0.0) for c in chunks]
    assert results[-1] is True and not any(results[:-1])
    assert asm.ready()
    assert asm.objects() == objs


def test_unvollstaendige_liste_ersetzt_die_alte_nicht():
    sender = _load_sender()
    asm = SnapshotAssembler()
    old = _objects(3)
    for c in sender.encode_chunks("obstacles", 1, 1, old):
        asm.feed(c, now=0.0)
    new = sender.encode_chunks("obstacles", 1, 2, _objects(700))
    for c in new[:-1]:                                  # letztes Paket fehlt
        asm.feed(c, now=1.0)
    assert asm.objects() == old


def test_neuer_sim_lauf_verwirft_alte_daten():
    sender = _load_sender()
    asm = SnapshotAssembler()
    for c in sender.encode_chunks("obstacles", 1, 5, _objects(3)):
        asm.feed(c)
    for c in sender.encode_chunks("grasp", 2, 1, _objects(1)):
        asm.feed(c)
    assert asm.gen == 2
    assert not asm.ready()                              # Hindernisse des neuen Laufs fehlen
    assert [o["name"] for o in asm.objects()] == ["teil_0"]


def test_unveraenderte_wiederholung_meldet_keine_aenderung():
    sender = _load_sender()
    asm = SnapshotAssembler()
    chunks = sender.encode_chunks("obstacles", 1, 1, _objects(10))
    assert [asm.feed(c) for c in chunks][-1] is True
    version = asm.version
    assert not any(asm.feed(c) for c in chunks)       # periodisches Wiederholen
    assert asm.version == version


def test_leere_liste_wird_gemeldet():
    sender = _load_sender()
    asm = SnapshotAssembler()
    (chunk,) = sender.encode_chunks("grasp", 1, 1, [])
    assert asm.feed(chunk) is True
    assert asm.lists["grasp"] == []


def test_protokoll_v1_wird_weiter_verstanden():
    asm = SnapshotAssembler()
    msg = json.dumps({"obstacles": _objects(2), "grasp": []}).encode()
    assert asm.feed(msg) is True
    assert len(asm.objects()) == 2


def test_kaputte_datagramme_werden_ignoriert():
    asm = SnapshotAssembler()
    for data in (b"", b"{", b"[]", b'{"v":2,"kind":"x"}',
                 b'{"v":2,"kind":"grasp","gen":1,"seq":1,"part":3,"parts":2,"items":[]}'):
        assert asm.feed(data) is False
    assert asm.objects() == []
