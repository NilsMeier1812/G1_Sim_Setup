#!/usr/bin/env python3
"""
Tests fuer build_env_scene.py - vor allem die NORMALISIERUNG.

Warum das wichtig ist: der Scene-Editor exportiert jedes per Maus angelegte
Objekt als eigenen <body> mit freiem Gelenk (<joint type="free"/>) und laesst
die Pose am Geom stehen. Ungefiltert faellt damit die halbe Umgebung beim Start
um. build_env_scene.py muss daraus die kanonische Form machen:

    Hindernis     -> <geom> direkt im worldbody (statisch)
    grasp_-Objekt -> <body name=X pos=..><freejoint/><geom name=X/></body>

Genau diese Form erwarten scene_objects.py (Sim) und scene_bridge.py (ROS).

Laufen lassen (braucht nur die Standardbibliothek):
    python3 test_build_env_scene.py
"""
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import build_env_scene as bes

HERE = Path(__file__).resolve().parent

# So sieht ein Export des Editors wirklich aus (2 Primitive im Free-Body,
# 1 Mesh statisch daneben, Boden + Kamera + Keyframe als Beifang).
EDITOR_EXPORT = """
<mujoco model="MuJoCo Model">
  <compiler angle="radian"/>
  <size nkey="1"/>
  <asset>
    <mesh name="crate_mesh" content_type="model/stl" file="{mesh}"/>
  </asset>
  <worldbody>
    <geom name="crate" pos="1.6 0 0.15" type="mesh" mass="0" mesh="crate_mesh"/>
    <geom name="floor" size="2.5 2.5 0.01" pos="0 0 -0.01" type="plane"/>
    <camera name="camera" pos="1 0 1.5"/>
    <light pos="0 0 5" dir="0 0 -1"/>
    <body name="box_0001_body_0">
      <joint type="free"/>
      <geom name="box_0001" size="0.05 0.05 0.05" pos="1 0.6 0.15" type="box" mass="0.2"/>
    </body>
    <body name="grasp_apfel_0002_body_1">
      <joint type="free"/>
      <geom name="grasp_apfel_0002" size="0.05" pos="0.5 0 0.9" mass="0"/>
    </body>
  </worldbody>
  <keyframe><key name="home" qpos="0 0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe>
</mujoco>
"""


def merge(env, env_dir=None):
    """Umgebung (XML-Text oder Element) in eine frische Basis mischen."""
    env_root = ET.fromstring(env) if isinstance(env, str) else env
    warnings = []
    _mj, asset, wb = bes.build_base("g1_29dof.xml", "test")
    counts = bes.merge_environment(env_root, asset, wb, env_dir or HERE / "scenes",
                                   warnings)
    return wb, asset, warnings, counts


def geoms(wb):
    """Statische Hindernisse: direkte <geom>-Kinder ohne den Basis-Boden."""
    return {g.get("name"): g for g in wb.findall("geom") if g.get("name") != "floor"}


def bodies(wb):
    return {b.get("name"): b for b in wb.findall("body")}


class TestEditorExport(unittest.TestCase):
    """Der Regelfall: was der Editor wirklich schreibt."""

    def setUp(self):
        mesh = HERE / "meshes" / "sample_crate.stl"
        self.wb, self.asset, self.warnings, self.counts = merge(
            EDITOR_EXPORT.format(mesh=mesh))

    def test_free_body_wird_statisches_hindernis(self):
        # Der Editor haengt JEDES Primitiv in einen Free-Body. Ohne Umbau wuerde
        # das Objekt beim Start umfallen.
        g = geoms(self.wb)
        self.assertIn("box_0001", g)
        self.assertEqual(g["box_0001"].get("type"), "box")
        # Pose des inneren Geoms muss an die Welt-Pose durchgereicht werden.
        self.assertEqual([float(v) for v in g["box_0001"].get("pos").split()],
                         [1.0, 0.6, 0.15])
        self.assertNotIn("box_0001_body_0", bodies(self.wb))

    def test_grasp_objekt_wird_freier_koerper(self):
        b = bodies(self.wb)
        self.assertIn("grasp_apfel_0002", b)
        body = b["grasp_apfel_0002"]
        self.assertIsNotNone(body.find("freejoint"))
        # Die Pose gehoert an den BODY (die Sim liest die Live-Pose ueber den
        # Body-Namen), das Geom sitzt bei 0/Identity darin.
        self.assertEqual([float(v) for v in body.get("pos").split()], [0.5, 0.0, 0.9])
        inner = body.find("geom")
        self.assertEqual(inner.get("name"), "grasp_apfel_0002")
        self.assertIsNone(inner.get("pos"))

    def test_grasp_ohne_masse_bekommt_keine_null_masse(self):
        # mass="0" + freejoint = "mass and inertia of moving bodies must be
        # larger than mass" beim Kompilieren.
        inner = bodies(self.wb)["grasp_apfel_0002"].find("geom")
        self.assertIsNone(inner.get("mass"))

    def test_geomtyp_wird_immer_geschrieben(self):
        # MuJoCos Default ist "sphere"; der Editor laesst type= bei Kugeln weg.
        inner = bodies(self.wb)["grasp_apfel_0002"].find("geom")
        self.assertEqual(inner.get("type"), "sphere")

    def test_boden_und_licht_kommen_aus_der_basis(self):
        planes = [g for g in self.wb.findall("geom") if g.get("type") == "plane"]
        self.assertEqual(len(planes), 1)
        self.assertEqual(planes[0].get("name"), "floor")
        self.assertEqual(len(self.wb.findall("light")), 1)

    def test_kamera_bleibt_erhalten(self):
        self.assertEqual([c.get("name") for c in self.wb.findall("camera")], ["camera"])

    def test_meshpfad_wird_repo_relativ(self):
        mesh = self.asset.find("mesh")
        self.assertFalse(Path(mesh.get("file")).is_absolute())
        self.assertTrue((bes.G1_MESHDIR / mesh.get("file")).resolve().is_file())

    def test_zaehler(self):
        n_obstacles, n_grasp = self.counts
        self.assertEqual((n_obstacles, n_grasp), (2, 1))  # crate + box, apfel


class TestRobustheit(unittest.TestCase):

    def test_fehlendes_mesh_fliegt_samt_objekt_raus(self):
        xml = """
        <mujoco>
          <asset><mesh name="weg" file="/gibt/es/nicht.stl"/></asset>
          <worldbody>
            <geom name="kiste" type="mesh" mesh="weg" pos="1 0 0"/>
            <geom name="wuerfel" type="box" size="0.1 0.1 0.1" pos="2 0 0"/>
          </worldbody>
        </mujoco>"""
        wb, asset, warnings, _ = merge(xml)
        self.assertNotIn("kiste", geoms(wb))       # sonst laedt die Szene nicht
        self.assertIn("wuerfel", geoms(wb))        # der Rest bleibt nutzbar
        self.assertEqual(asset.findall("mesh"), [])
        self.assertTrue(any("nicht gefunden" in w for w in warnings))

    def test_namenskollision_mit_der_basis(self):
        xml = """
        <mujoco><worldbody>
          <geom name="floor" type="box" size="0.1 0.1 0.1" pos="1 0 0"/>
        </worldbody></mujoco>"""
        wb, _asset, _w, _c = merge(xml)
        names = [g.get("name") for g in wb.findall("geom")]
        self.assertEqual(names.count("floor"), 1)  # der Boden der BASIS
        self.assertIn("floor_2", names)

    def test_doppelte_namen_werden_eindeutig(self):
        xml = """
        <mujoco><worldbody>
          <geom name="kiste" type="box" size="0.1 0.1 0.1" pos="1 0 0"/>
          <geom name="kiste" type="box" size="0.1 0.1 0.1" pos="2 0 0"/>
        </worldbody></mujoco>"""
        wb, _asset, _w, _c = merge(xml)
        self.assertEqual(sorted(geoms(wb)), ["kiste", "kiste_2"])

    def test_namen_werden_bereinigt(self):
        # Blueprint-Pfade des Editors fangen mit '/' an.
        xml = """
        <mujoco><worldbody><body name="/gruppe">
          <geom name="/mein objekt" type="box" size="0.1 0.1 0.1"/>
        </body></worldbody></mujoco>"""
        wb, _asset, _w, _c = merge(xml)
        self.assertIn("mein_objekt", geoms(wb))

    def test_verschachtelte_gruppe_wird_verrechnet(self):
        # Gruppe 90 Grad um z gedreht, Objekt 1 m in x -> landet bei y=1.
        s = 2 ** -0.5
        xml = f"""
        <mujoco><worldbody>
          <body name="gruppe" pos="0 0 0.5" quat="{s} 0 0 {s}">
            <geom name="objekt" type="box" size="0.1 0.1 0.1" pos="1 0 0"/>
          </body>
        </worldbody></mujoco>"""
        wb, _asset, _w, _c = merge(xml)
        pos = [float(v) for v in geoms(wb)["objekt"].get("pos").split()]
        self.assertAlmostEqual(pos[0], 0.0, places=5)
        self.assertAlmostEqual(pos[1], 1.0, places=5)
        self.assertAlmostEqual(pos[2], 0.5, places=5)

    def test_grasp_am_body_zaehlt_auch(self):
        # Editor: Body heisst "<geom>_body_<n>" - grasp_ kann an beiden stehen.
        xml = """
        <mujoco><worldbody>
          <body name="grasp_kiste_body_0" pos="1 0 0.3">
            <freejoint/>
            <geom name="irgendwas" type="box" size="0.1 0.1 0.1"/>
          </body>
        </worldbody></mujoco>"""
        wb, _asset, _w, counts = merge(xml)
        self.assertEqual(counts, (0, 1))
        self.assertIn("irgendwas", bodies(wb))

    def test_artikulierter_koerper_bleibt_unangetastet(self):
        xml = """
        <mujoco><worldbody>
          <body name="tuer" pos="1 0 0">
            <joint name="scharnier" type="hinge" axis="0 0 1"/>
            <geom name="blatt" type="box" size="0.4 0.02 1.0"/>
          </body>
        </worldbody></mujoco>"""
        wb, _asset, warnings, _c = merge(xml)
        body = bodies(wb)["tuer"]
        self.assertIsNotNone(body.find("joint"))
        self.assertTrue(any("eigene Gelenke" in w for w in warnings))

    def test_ignorierte_abschnitte_werden_gemeldet(self):
        xml = """
        <mujoco>
          <actuator/>
          <worldbody><geom name="k" type="box" size="0.1 0.1 0.1"/></worldbody>
        </mujoco>"""
        _wb, _asset, warnings, _c = merge(xml)
        self.assertTrue(any("<actuator>" in w for w in warnings))


class TestSTLBehandlung(unittest.TestCase):
    """MuJoCo laedt nur binaeres STL ('perhaps this is an ASCII file?').

    Ohne diese Pruefung/Reparatur reisst EIN schlechtes Mesh die ganze
    Umgebung mit sich (MuJoCo bricht das Kompilieren der GESAMTEN Szene ab,
    egal wie viele anderen Objekte fehlerfrei sind).
    """

    def setUp(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)

    def _mesh_path(self, data: bytes, name="mesh.stl") -> Path:
        p = Path(self._tmpdir.name) / name
        p.write_bytes(data)
        return p

    @staticmethod
    def _binary_stl_bytes(ntri=1):
        # Minimalste gueltige binaere STL: 80-Byte-Header + Face-Anzahl +
        # ntri * 50 Byte-Dreiecke (Dreiecksinhalt ist fuer den Groessen-Check
        # irrelevant, Nullen reichen).
        return b"\0" * 80 + struct.pack("<I", ntri) + b"\0" * (50 * ntri)

    def test_erkennt_binaere_stl(self):
        p = self._mesh_path(self._binary_stl_bytes(3))
        self.assertTrue(bes.is_valid_binary_stl(p))

    def test_erkennt_ascii_stl(self):
        ascii_stl = (b"solid test\n facet normal 0 0 1\n outer loop\n"
                    b"  vertex 0 0 0\n vertex 1 0 0\n vertex 0 1 0\n"
                    b" endloop\nendfacet\nendsolid test\n")
        p = self._mesh_path(ascii_stl)
        self.assertFalse(bes.is_valid_binary_stl(p))

    def test_erkennt_kaputten_face_count(self):
        # Groesse passt nicht zur behaupteten Face-Anzahl im Header.
        bad = b"\0" * 80 + struct.pack("<I", 5) + b"\0" * 10
        p = self._mesh_path(bad)
        self.assertFalse(bes.is_valid_binary_stl(p))

    def test_resolve_mesh_repariert_ascii_stl_automatisch(self):
        try:
            import trimesh
        except ImportError:
            self.skipTest("trimesh nicht installiert")
        box = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
        p = self._mesh_path(b"", name="ascii_box.stl")
        box.export(str(p), file_type="stl_ascii")
        self.assertFalse(bes.is_valid_binary_stl(p))

        warnings = []
        result = bes._resolve_mesh(str(p), p.parent, warnings)
        # /tmp liegt ausserhalb von unitree_mujoco/ -> wird nach
        # meshes/imported/ kopiert; DORT muss die Reparatur sichtbar sein.
        self.assertIsNotNone(result)
        self.addCleanup(lambda: result.unlink(missing_ok=True))
        self.assertTrue(bes.is_valid_binary_stl(result))
        self.assertTrue(any("ASCII-STL" in w for w in warnings))

    def _without_trimesh(self):
        """trimesh-Importfehler simulieren (ohne das Modul zu deinstallieren)."""
        import builtins
        real_import = builtins.__import__

        def blocked(name, *a, **k):
            if name == "trimesh":
                raise ImportError("blockiert fuer Test")
            return real_import(name, *a, **k)

        builtins.__import__ = blocked
        self.addCleanup(lambda: setattr(builtins, "__import__", real_import))

    def test_ascii_reparatur_braucht_kein_trimesh(self):
        ascii_stl = (b"solid test\n facet normal 0 0 1\n outer loop\n"
                     b"  vertex 0 0 0\n vertex 1 0 0\n vertex 0 1 0\n"
                     b" endloop\nendfacet\nendsolid test\n")
        p = self._mesh_path(ascii_stl, name="ascii_plain.stl")
        self._without_trimesh()
        warnings = []
        self.assertTrue(bes.convert_stl_to_binary(p, warnings))
        self.assertTrue(bes.is_valid_binary_stl(p))
        self.assertTrue(any("ASCII-STL" in w for w in warnings))

    def test_leere_stl_laesst_objekt_weg_statt_abzustuerzen(self):
        p = self._mesh_path(b"solid x\nendsolid x\n")
        self._without_trimesh()
        warnings = []
        self.assertFalse(bes.convert_stl_to_binary(p, warnings))
        self.assertTrue(any(p.name in w for w in warnings))

    def test_zu_viele_faces_werden_gemeldet(self):
        # Header behauptet mehr Dreiecke als MuJoCo erlaubt, Groesse passt dazu.
        n = bes._MJ_STL_MAX_FACES + 1
        p = self._mesh_path(b"\0" * 80 + struct.pack("<I", n) + b"\0" * (n * 50),
                            name="riesig.stl")
        self.assertFalse(bes.is_valid_binary_stl(p))
        warnings = []
        self.assertFalse(bes.convert_stl_to_binary(p, warnings))
        self.assertTrue(any("Dreiecke" in w for w in warnings))

    def test_fehlerhaftes_mesh_reisst_nicht_die_ganze_umgebung_mit(self):
        bad = self._mesh_path(b"solid x\nendsolid x\n", name="bad.stl")
        # Das Mesh liegt ausserhalb des Repos -> merge() kopiert es nach
        # meshes/imported/. Diese Kopie hinterher wieder wegraeumen.
        self.addCleanup((bes.IMPORTED_MESHES_DIR / bad.name).unlink, True)
        xml = f"""
        <mujoco>
          <asset><mesh name="bad_mesh" file="{bad}"/></asset>
          <worldbody>
            <geom name="kaputt" type="mesh" mesh="bad_mesh" pos="1 0 0"/>
            <geom name="ok" type="box" size="0.1 0.1 0.1" pos="2 0 0"/>
          </worldbody>
        </mujoco>"""
        import builtins
        real_import = builtins.__import__

        def blocked(name, *a, **k):
            if name == "trimesh":
                raise ImportError("blockiert fuer Test")
            return real_import(name, *a, **k)

        builtins.__import__ = blocked
        try:
            wb, _asset, warnings, _counts = merge(xml, env_dir=Path(self._tmpdir.name))
        finally:
            builtins.__import__ = real_import
        self.assertNotIn("kaputt", geoms(wb))
        self.assertIn("ok", geoms(wb))
        self.assertTrue(any("fehlendes Mesh" in w for w in warnings))


class TestNamenUndPfade(unittest.TestCase):

    def test_sanitize_name(self):
        self.assertEqual(bes.sanitize_name("/box_0001"), "box_0001")
        self.assertEqual(bes.sanitize_name("mein objekt"), "mein_objekt")
        self.assertEqual(bes.sanitize_name("a/b"), "a_b")
        self.assertEqual(bes.sanitize_name("   "), "")

    def test_is_grasp_name(self):
        self.assertTrue(bes.is_grasp_name("grasp_apfel"))
        self.assertTrue(bes.is_grasp_name("Grasp_Apfel"))
        self.assertFalse(bes.is_grasp_name("apfel"))
        self.assertFalse(bes.is_grasp_name("kiste_grasp"))

    def test_resolve_env_path_akzeptiert_kurzform(self):
        starter = bes.SCENES_DIR / "environment_starter" / "umgebung.xml"
        if not starter.is_file():
            self.skipTest("Beispiel-Umgebung nicht vorhanden")
        for arg in ("environment_starter", "scenes/environment_starter",
                    str(starter.parent), str(starter)):
            self.assertEqual(bes.resolve_env_path(arg), starter.resolve())
        self.assertIn("environment_starter", bes.available_envs())


class TestStarterUmgebung(unittest.TestCase):
    """Von Hand geschriebene Umgebungen (kanonische Form) bleiben unveraendert."""

    def test_starter_bleibt_statisch(self):
        starter = bes.SCENES_DIR / "environment_starter" / "umgebung.xml"
        if not starter.is_file():
            self.skipTest("Beispiel-Umgebung nicht vorhanden")
        wb, _asset, _w, counts = merge(bes.parse_environment(starter),
                                       env_dir=starter.parent)
        self.assertEqual(counts[1], 0)                      # keine Greif-Objekte
        # alles statisch (einzige Bodies: Kollisions-Zerlegung der Meshes)
        self.assertEqual({n for n in bodies(wb)
                          if not n.endswith(bes.COLLISION_BODY_SUFFIX)}, set())
        self.assertIn("box_demo", geoms(wb))


def _trimesh_ok() -> bool:
    try:
        import trimesh  # noqa: F401
        import vhacdx  # noqa: F401
    except ImportError:
        return False
    return True


class TestStartpunkt(unittest.TestCase):
    """g1_spawn: Startpunkt des G1 statt eines Hindernisses."""

    ENV = """
<mujoco>
  <worldbody>
    <body name="g1_spawn_body_0">
      <joint type="free"/>
      <geom name="g1_spawn" size="0.1 0.1 0.1" pos="1 2 0.1" type="box"
            quat="0.7071068 0 0 0.7071068"/>
    </body>
    <geom name="tisch" size="0.5 0.5 0.4" pos="2 0 0.4" type="box"/>
  </worldbody>
</mujoco>
"""

    def test_spawn_wird_startpunkt_und_kein_objekt(self):
        warnings, spawn = [], []
        _mj, asset, wb = bes.build_base("g1_29dof.xml", "test")
        counts = bes.merge_environment(ET.fromstring(self.ENV), asset, wb,
                                       HERE / "scenes", warnings, spawn_out=spawn)
        self.assertEqual(len(spawn), 1)
        x, y, yaw = spawn[0]
        self.assertAlmostEqual(x, 1.0, places=5)
        self.assertAlmostEqual(y, 2.0, places=5)
        self.assertAlmostEqual(yaw, 1.5707963, places=5)
        self.assertNotIn("g1_spawn", geoms(wb))
        self.assertEqual(bodies(wb), {})
        self.assertEqual(counts, (1, 0))                    # nur der Tisch

    def test_ohne_spawn_out_trotzdem_entfernt(self):
        wb, _asset, _w, counts = merge(self.ENV)
        self.assertNotIn("g1_spawn", geoms(wb))
        self.assertEqual(counts, (1, 0))


class TestKollisionsBits(unittest.TestCase):
    """Umgebungs-Geoms kollidieren mit Koerper (Bit 1) UND Haenden (Bit 2)."""

    ENV = """
<mujoco>
  <worldbody>
    <geom name="tisch" size="0.5 0.5 0.4" pos="2 0 0.4" type="box"/>
    <geom name="deko" size="0.1" pos="0 2 0.1" type="sphere" contype="0" conaffinity="0"/>
    <body name="grasp_box_body_0">
      <joint type="free"/>
      <geom name="grasp_box" size="0.05 0.05 0.05" pos="2 0 0.85" type="box"/>
    </body>
  </worldbody>
</mujoco>
"""

    def test_bits(self):
        wb, _asset, _w, _counts = merge(self.ENV)
        g = geoms(wb)
        self.assertEqual((g["tisch"].get("contype"), g["tisch"].get("conaffinity")),
                         (bes.ENV_COLLISION_BITS, bes.ENV_COLLISION_BITS))
        inner = bodies(wb)["grasp_box"].find("geom")
        self.assertEqual(inner.get("contype"), bes.ENV_COLLISION_BITS)
        # explizit gesetzte Werte (reine Optik) bleiben
        self.assertEqual((g["deko"].get("contype"), g["deko"].get("conaffinity")), ("0", "0"))

    def test_basis_bleibt_unangetastet(self):
        wb, _asset, _w, _counts = merge(self.ENV)
        floor = [x for x in wb.findall("geom") if x.get("name") == "floor"][0]
        self.assertIsNone(floor.get("contype"))


@unittest.skipUnless(_trimesh_ok(), "trimesh/vhacdx fehlen")
class TestKollisionsZerlegung(unittest.TestCase):
    """Statische STL-Hindernisse kollidieren ueber konvexe Teile (V-HACD)."""

    def _env(self, mesh_file, n=1):
        geoms_xml = "".join(
            f'<geom name="werk{i}" pos="{i} 0 0" type="mesh" mesh="werk_mesh"/>'
            for i in range(n))
        return (f'<mujoco><asset><mesh name="werk_mesh" file="{mesh_file}"/></asset>'
                f"<worldbody>{geoms_xml}</worldbody></mujoco>")

    def test_mesh_wird_zerlegt(self):
        mesh = bes.MESHES_DIR / "sample_crate.stl"
        if not mesh.is_file():
            self.skipTest("sample_crate.stl nicht vorhanden")
        wb, asset, _w, _counts = merge(self._env(mesh, n=2))
        g = geoms(wb)
        for name in ("werk0", "werk1"):
            self.assertEqual(g[name].get("contype"), "0")          # nur noch Optik
            col = bodies(wb)[name + bes.COLLISION_BODY_SUFFIX]
            parts = col.findall("geom")
            self.assertGreaterEqual(len(parts), 1)
            self.assertEqual(parts[0].get("group"), "3")
            self.assertEqual(parts[0].get("contype"), bes.ENV_COLLISION_BITS)
        # gleiches Mesh zweimal benutzt -> Teil-Assets nur EINMAL (MuJoCo
        # bricht bei doppelten Asset-Namen ab)
        names = [m.get("name") for m in asset.findall("mesh")]
        self.assertEqual(len(names), len(set(names)))

    def test_cad_import_meshes_werden_nicht_nochmal_zerlegt(self):
        src = bes.MESHES_DIR / "sample_crate.stl"
        if not src.is_file():
            self.skipTest("sample_crate.stl nicht vorhanden")
        # Unter unitree_mujoco/ anlegen - sonst kopiert der Generator das Mesh
        # nach meshes/imported/ und das Manifest daneben fehlt.
        with tempfile.TemporaryDirectory(dir=bes.MESHES_DIR, prefix=".test_cad_") as td:
            d = Path(td)
            (d / "teil.stl").write_bytes(src.read_bytes())
            (d / "cad_import.json").write_text(
                '{"meshes": {"teil.stl": {"rgba": [1, 0, 0, 1], "collision": true, '
                '"visual": true}}}', encoding="utf-8")
            wb, _asset, _w, _counts = merge(self._env(d / "teil.stl"))
        self.assertNotIn("werk0" + bes.COLLISION_BODY_SUFFIX, bodies(wb))
        self.assertEqual(geoms(wb)["werk0"].get("contype"), bes.ENV_COLLISION_BITS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
