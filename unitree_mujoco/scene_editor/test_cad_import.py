#!/usr/bin/env python3
"""Tests fuer cad_import.py (CAD-/Mesh-Import -> Einzelteil-Meshes + Umgebung).

Gestaffelt nach installierten Paketen - was fehlt, wird uebersprungen:
  * nur Standardbibliothek: Namen, Quaternionen, Manifest -> XML-Defaults
  * numpy:   Netz teilen, STL schreiben, Geschlossenheit, Spiegelung
  * trimesh: kompletter Import einer Mesh-Szene (Posen, Farben, Deduplizierung)
  * OCP:     STEP-Import (sample_bracket.step)
  * mujoco:  erzeugte Umgebung laedt wirklich
  * coacd:   konkave Teile werden zerlegt

    python3 test_cad_import.py            (im Editor-venv: alles)
"""
import json
import math
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from importlib.util import find_spec
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cad_import as ci  # noqa: E402
import mesh_utils  # noqa: E402

HAVE_NUMPY = find_spec("numpy") is not None
HAVE_TRIMESH = ci.trimesh_available()
HAVE_OCC = ci.occ_available()
HAVE_MUJOCO = find_spec("mujoco") is not None
HAVE_COACD = ci.coacd_available()


class _TmpDir(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="cad_import_test_"))

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# Nur Standardbibliothek
# ---------------------------------------------------------------------------
class TestNamenUndMathe(unittest.TestCase):
    def test_namen_werden_mujoco_tauglich(self):
        self.assertEqual(ci.sanitize_name("Schutztür (links)/Teil 1"), "Schutztuer_links_Teil_1")
        self.assertEqual(ci.sanitize_name("8626_000(1)_M51(1)"), "8626_000_1_M51_1")
        self.assertEqual(ci.sanitize_name("///", "x"), "x")

    def test_generische_cad_namen(self):
        for n in ("SOLID", "COMPOUND", "", "Open CASCADE STEP translator 7.8 1.1"):
            self.assertTrue(ci._is_generic(n), n)
        self.assertFalse(ci._is_generic("Zellenwand"))

    def test_eindeutig_ohne_gross_klein(self):
        used = set()
        self.assertEqual([ci._unique(n, used) for n in ("Teil", "teil", "TEIL")],
                         ["Teil", "teil_2", "TEIL_3"])

    def test_quaternion(self):
        self.assertEqual(ci._mat_to_quat([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), [1.0, 0.0, 0.0, 0.0])
        q = ci._mat_to_quat([[0, -1, 0], [1, 0, 0], [0, 0, 1]])        # 90 Grad um z
        for a, b in zip(q, (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))):
            self.assertAlmostEqual(a, b)
        q = ci._mat_to_quat([[1, 0, 0], [0, -1, 0], [0, 0, -1]])       # 180 Grad um x
        self.assertAlmostEqual(abs(q[1]), 1.0)

    def test_srgb(self):
        self.assertAlmostEqual(ci._srgb(0.44), 0.694, places=2)      # smoke gray aus NX
        self.assertEqual(ci._srgb(0.0), 0.0)
        self.assertAlmostEqual(ci._srgb(1.0), 1.0)

    def test_unbekanntes_format_klartext(self):
        p = Path(tempfile.mkdtemp()) / "teil.jt"
        p.write_bytes(b"x")
        with self.assertRaises(ci.ImportFailed) as ctx:
            ci.import_file(p)
        self.assertIn("STEP", str(ctx.exception))


class TestManifestDefaults(_TmpDir):
    """Farbe/Kollision/shell aus dem Manifest ins XML (Editor-Durchlauf)."""

    def _manifest(self):
        d = self.dir / "meshes" / "cad" / "zelle"
        d.mkdir(parents=True)
        (d / ci.MANIFEST_NAME).write_text(json.dumps({"meshes": {
            "wand.stl": {"rgba": [0.1, 0.2, 0.3, 1], "collision": False, "visual": True,
                         "shell": True},
            "wand__huelle.stl": {"rgba": [0.1, 0.2, 0.3, 1], "collision": True,
                                 "visual": False, "shell": False},
        }}))
        return d

    def test_fehlende_angaben_werden_ergaenzt(self):
        self._manifest()
        scenes = self.dir / "scenes"
        root = ET.fromstring(
            '<mujoco><asset>'
            '<mesh name="w" file="../meshes/cad/zelle/wand.stl"/>'
            '<mesh name="h" file="../meshes/cad/zelle/wand__huelle.stl"/>'
            '<mesh name="fremd" file="../meshes/kiste.stl"/></asset>'
            '<worldbody><geom name="a" type="mesh" mesh="w"/>'
            '<geom name="b" type="mesh" mesh="h"/>'
            '<geom name="c" type="mesh" mesh="w" rgba="1 0 0 1"/>'
            '<geom name="d" type="mesh" mesh="fremd"/></worldbody></mujoco>')
        ci.apply_mesh_defaults(root, scenes)
        g = {e.get("name"): e for e in root.iter("geom")}
        self.assertEqual(g["a"].get("rgba"), "0.1 0.2 0.3 1")
        self.assertEqual((g["a"].get("contype"), g["a"].get("conaffinity")), ("0", "0"))
        self.assertEqual(g["b"].get("group"), ci.COLLISION_GROUP)
        self.assertIsNone(g["b"].get("contype"))
        self.assertEqual(g["c"].get("rgba"), "1 0 0 1")              # von Hand gesetzt bleibt
        self.assertIsNone(g["d"].get("rgba"))                         # kein CAD-Mesh
        m = {e.get("name"): e for e in root.iter("mesh")}
        self.assertEqual(m["w"].get("inertia"), "shell")
        self.assertIsNone(m["h"].get("inertia"))


# ---------------------------------------------------------------------------
# numpy
# ---------------------------------------------------------------------------
def _box(sx=1.0, sy=1.0, sz=1.0):
    import numpy as np
    v = np.array([[x, y, z] for x in (0, sx) for y in (0, sy) for z in (0, sz)], dtype=float)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f


@unittest.skipUnless(HAVE_NUMPY, "numpy fehlt")
class TestNetze(_TmpDir):
    def test_split_haelt_grenze_ein_und_verliert_nichts(self):
        import numpy as np
        rng = np.random.default_rng(1)
        v = rng.random((3000, 3))
        f = rng.integers(0, 3000, size=(1000, 3))
        parts = ci.split_faces(v, f, 300)
        self.assertTrue(all(len(pf) <= 300 for _, pf in parts))
        self.assertEqual(sum(len(pf) for _, pf in parts), 1000)
        orig = {tuple(np.round(v[t].ravel(), 9)) for t in f}
        back = {tuple(np.round(pv[t].ravel(), 9)) for pv, pf in parts for t in pf}
        self.assertEqual(orig, back)

    def test_write_stl_ist_mujoco_tauglich(self):
        v, f = _box()
        p = self.dir / "box.stl"
        self.assertEqual(ci.write_stl(p, v, f), 12)
        self.assertTrue(mesh_utils.is_valid_binary_stl(p))

    def test_geschlossen_oder_shell(self):
        v, f = _box()
        self.assertTrue(ci.mesh_is_solid(v, f))
        self.assertFalse(ci.mesh_is_solid(v, f[:10]))                 # Deckel fehlt
        flat, _ = _box(1, 1, 0)
        self.assertFalse(ci.mesh_is_solid(flat, f))                   # kein Volumen

    def test_spiegelung_wird_abgetrennt(self):
        import numpy as np
        M = np.diag([1.0, -1.0, 1.0, 1.0])
        M[:3, 3] = [1, 2, 3]
        R, t, S = ci._split_rigid(M)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        np.testing.assert_allclose(R @ S, M[:3, :3], atol=1e-12)
        np.testing.assert_allclose(t, [1, 2, 3])


# ---------------------------------------------------------------------------
# trimesh: kompletter Import
# ---------------------------------------------------------------------------
def _world_vertices(result):
    """Alle sichtbaren Dreiecke der Umgebung in Weltkoordinaten (CAD-Frame)."""
    import numpy as np
    import trimesh
    out = []
    for inst in result.parts:
        m = trimesh.load_mesh(str(result.out_dir / inst.mesh), process=False)
        w, x, y, z = inst.quat
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        out.append(np.asarray(m.vertices) @ R.T + np.asarray(inst.pos))
    return np.vstack(out)


@unittest.skipUnless(HAVE_TRIMESH, "trimesh fehlt")
class TestMeshImport(_TmpDir):
    def _scene(self):
        """Szene: Kiste 2x (einmal gedreht+verschoben, einmal GESPIEGELT), Platte."""
        import numpy as np
        import trimesh
        box = trimesh.Trimesh(*_box(0.4, 0.2, 0.1), process=False)
        box.visual.face_colors = [200, 30, 30, 255]
        plate = trimesh.Trimesh(*_box(1.0, 1.0, 0.02), process=False)
        scene = trimesh.Scene()
        T1 = trimesh.transformations.rotation_matrix(0.7, [0, 0, 1])
        T1[:3, 3] = [2, 1, 0.5]
        mirror = np.diag([1.0, -1.0, 1.0, 1.0])
        mirror[:3, 3] = [-1, 0, 0.3]
        scene.add_geometry(box, node_name="kiste_a", geom_name="kiste", transform=T1)
        scene.add_geometry(box, node_name="kiste_b", geom_name="kiste", transform=mirror)
        scene.add_geometry(plate, node_name="platte", geom_name="platte")
        src = self.dir / "szene.glb"
        scene.export(src)
        return src, scene

    def test_geometrie_posen_farben(self):
        import numpy as np
        src, scene = self._scene()
        res = ci.import_file(src, "szene", out_root=self.dir / "cad", log=lambda m: None)
        self.assertEqual(len(res.parts), 3)
        names = sorted(i.name for i in res.parts)
        self.assertEqual(names, ["kiste_a", "kiste_b", "platte"])
        # Jede STL ist auf ihre Bounding-Box-Mitte zentriert (Abnehmer rechnen so).
        for mf in res.meshes.values():
            lo, hi = mesh_utils.read_binary_stl_bounds(res.out_dir / mf.file)
            for a, b in zip(lo, hi):
                self.assertAlmostEqual(a + b, 0.0, places=5)
        # Gespiegelte Kiste braucht ein eigenes Mesh, die gedrehte nicht.
        self.assertEqual(len({i.mesh for i in res.parts}), 3)
        # Rekonstruktion == Original (inkl. Spiegelung).
        orig = scene.to_mesh().vertices
        back = _world_vertices(res)
        np.testing.assert_allclose(back.min(axis=0), orig.min(axis=0), atol=1e-5)
        np.testing.assert_allclose(back.max(axis=0), orig.max(axis=0), atol=1e-5)
        key = lambda a: {tuple(r) for r in np.round(a, 4)}         # noqa: E731
        self.assertEqual(key(back), key(orig))
        kiste = next(i for i in res.parts if i.name == "kiste_a")
        self.assertAlmostEqual(kiste.rgba[0], 200 / 255, places=2)
        # Manifest ist zugleich Cache.
        again = ci.import_file(src, "szene", out_root=self.dir / "cad", log=lambda m: None)
        self.assertTrue(again.from_cache)
        self.assertEqual([i.name for i in again.instances], [i.name for i in res.instances])

    def test_farbe_gehoert_zur_datei(self):
        """Der Editor kennt Farben nur je Mesh-Datei (Manifest) - gleiche
        Geometrie in zwei Farben braucht also zwei Dateien."""
        import numpy as np
        import trimesh
        scene = trimesh.Scene()
        for i, c in enumerate(([200, 30, 30, 255], [30, 200, 30, 255], [200, 30, 30, 255])):
            box = trimesh.Trimesh(*_box(0.3, 0.2, 0.1), process=False)
            box.visual.face_colors = c
            T = np.eye(4)
            T[:3, 3] = [i * 0.5, 0, 0]
            scene.add_geometry(box, node_name=f"k{i}", geom_name=f"k{i}", transform=T)
        src = self.dir / "kisten.glb"
        scene.export(src)
        res = ci.import_file(src, "kisten", out_root=self.dir / "cad", log=lambda m: None)
        by_name = {i.name: i for i in res.parts}
        self.assertEqual(by_name["k0"].mesh, by_name["k2"].mesh)       # rot + rot: eine Datei
        self.assertNotEqual(by_name["k0"].mesh, by_name["k1"].mesh)    # rot + gruen: zwei
        for inst in res.parts:
            self.assertEqual(res.meshes[inst.mesh].rgba, inst.rgba)    # Editor sieht die Farbe

    def test_zu_grosses_netz_wird_geteilt(self):
        import numpy as np
        import trimesh
        n = mesh_utils.MJ_MAX_FACES + 5000
        rng = np.random.default_rng(0)
        v = rng.random((n + 2, 3))
        f = np.stack([np.arange(n), np.arange(n) + 1, np.arange(n) + 2], axis=1)
        src = self.dir / "riesig.stl"
        trimesh.Trimesh(v, f, process=False).export(src)
        res = ci.import_file(src, "riesig", out_root=self.dir / "cad", log=lambda m: None)
        self.assertGreaterEqual(len(res.parts), 2)
        for mf in res.meshes.values():
            self.assertTrue(mesh_utils.is_valid_binary_stl(res.out_dir / mf.file))
        self.assertEqual(sum(res.meshes[i.mesh].faces for i in res.parts), n)

    def test_umgebung_und_platzierung(self):
        src, _ = self._scene()
        res = ci.import_file(src, "szene", out_root=self.dir / "cad", log=lambda m: None)
        xml = ci.write_environment(res, self.dir / "scenes" / "szene.xml", place="auto")
        root = ET.parse(xml).getroot()
        body = root.find("worldbody/body")
        self.assertEqual(body.get("name"), "szene")
        self.assertEqual(len(body.findall("geom")), 3)
        for m in root.iter("mesh"):
            self.assertTrue((xml.parent / m.get("file")).is_file(), m.get("file"))
        # auto: Unterkante auf z=0, der G1 (Ursprung) steht frei
        off = res.placement_offset("auto")
        self.assertAlmostEqual(res.bbox_min[2] + off[2], 0.0, places=5)
        self.assertEqual(res.spawn_conflicts(off), [])
        self.assertEqual(res.placement_offset("cad"), [0.0, 0.0, 0.0])

    def test_kleines_objekt_kommt_vor_den_g1(self):
        import trimesh
        src = self.dir / "kiste.stl"
        trimesh.Trimesh(*_box(0.4, 0.3, 0.2), process=False).export(src)
        res = ci.import_file(src, "kiste", out_root=self.dir / "cad", log=lambda m: None)
        off = res.placement_offset("auto")
        self.assertAlmostEqual(res.bbox_min[0] + off[0], 1.0, places=5)
        self.assertAlmostEqual(res.bbox_min[2] + off[2], 0.0, places=5)
        self.assertEqual(res.spawn_conflicts(off), [])
        # "cad": Kiste steht im Ursprung -> Warnung
        self.assertEqual(res.spawn_conflicts(res.placement_offset("cad")), ["kiste"])

    def test_bodenplatte_bestimmt_den_boden(self):
        import numpy as np
        import trimesh
        # Zelle: Bodenplatte 4x4 m (Oberkante z=0) + Anker, der 0.1 m tiefer reicht
        platte = trimesh.Trimesh(*_box(4, 4, 0.02), process=False)
        platte.apply_translation([0, 0, -0.02])
        anker = trimesh.Trimesh(*_box(0.2, 0.2, 0.3), process=False)
        anker.apply_translation([1, 1, -0.1])
        scene = trimesh.Scene()
        scene.add_geometry(platte, node_name="boden", geom_name="boden")
        scene.add_geometry(anker, node_name="anker", geom_name="anker")
        src = self.dir / "zelle.glb"
        scene.export(src)
        res = ci.import_file(src, "zelle", out_root=self.dir / "cad", log=lambda m: None)
        self.assertAlmostEqual(res.ground_z(), 0.0, places=5)
        off = res.placement_offset("auto")
        self.assertAlmostEqual(off[2], 0.0, places=5)
        # G1 steht in der Zelle, nicht im Anker
        self.assertTrue(0 < -off[0] < 4 and 0 < -off[1] < 4, off)
        self.assertEqual(res.spawn_conflicts(off), [])
        np.testing.assert_allclose(res.placement_offset("floor")[:2], [0, 0])

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco fehlt")
    def test_umgebung_laedt_in_mujoco(self):
        import mujoco
        src, _ = self._scene()
        res = ci.import_file(src, "szene", out_root=self.dir / "cad", log=lambda m: None)
        xml = ci.write_environment(res, self.dir / "scenes" / "szene.xml")
        model = mujoco.MjModel.from_xml_path(str(xml))
        self.assertEqual(model.ngeom, 3)


@unittest.skipUnless(HAVE_TRIMESH and HAVE_COACD, "trimesh/coacd fehlen")
class TestKonkav(_TmpDir):
    def test_l_wand_wird_zerlegt(self):
        import numpy as np
        import trimesh
        # L-foermige Wand 4 x 4 m, 0.3 m dick, 2 m hoch: Huelle ~ 16 m^3, Wand ~ 4.6 m^3
        # (so wie "Zellenwand" im DC-Demonstrator). Grundriss als Polygon mit
        # Punkt 6 auf der linken Kante, damit beide Deckel-Rechtecke passen.
        xy = [(0, 0), (4, 0), (4, .3), (.3, .3), (.3, 4), (0, 4), (0, .3)]
        v = np.array([(x, y, z) for z in (0.0, 2.0) for x, y in xy])
        n = len(xy)
        caps = [(0, 1, 2), (0, 2, 3), (0, 3, 6), (6, 3, 4), (6, 4, 5)]
        f = [(a, c, b) for a, b, c in caps] + [(a + n, b + n, c + n) for a, b, c in caps]
        for i in range(n):
            j = (i + 1) % n
            f += [(i, j, j + n), (i, j + n, i + n)]
        wand = trimesh.Trimesh(v, np.array(f))
        trimesh.repair.fix_normals(wand)
        self.assertTrue(wand.is_watertight)
        src = self.dir / "wand.stl"
        wand.export(src)
        res = ci.import_file(src, "wand", out_root=self.dir / "cad", log=lambda m: None)
        hulls = [i for i in res.instances if not i.visual]
        self.assertGreaterEqual(len(hulls), 2)
        self.assertFalse(res.parts[0].collision)          # Optik-Mesh kollidiert nicht mehr
        self.assertTrue(all(h.part == res.parts[0].name for h in hulls))


@unittest.skipUnless(HAVE_OCC and HAVE_NUMPY, "OpenCascade (cadquery-ocp) fehlt")
class TestStep(_TmpDir):
    def test_beispiel_step(self):
        src = ci.MESHES_DIR / "sample_bracket.step"
        if not src.is_file():
            self.skipTest("sample_bracket.step fehlt")
        res = ci.import_file(src, "bracket", out_root=self.dir / "cad", log=lambda m: None)
        self.assertGreaterEqual(len(res.parts), 1)
        size = [h - lo for lo, h in zip(res.bbox_min, res.bbox_max)]
        self.assertTrue(0.05 < max(size) < 1.0, size)          # mm -> m umgerechnet
        for i in res.parts:
            self.assertFalse(ci._is_generic(i.name), i.name)
        xml = ci.write_environment(res, self.dir / "scenes" / "bracket.xml")
        if HAVE_MUJOCO:
            import mujoco
            mujoco.MjModel.from_xml_path(str(xml))


if __name__ == "__main__":
    unittest.main(verbosity=2)
