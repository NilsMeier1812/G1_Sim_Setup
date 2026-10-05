#!/usr/bin/env python3
"""Tests fuer env_store.py - Umgebungen als eigenstaendige Ordner.

Nur Standardbibliothek. scenes/ und meshes/ werden auf ein Temp-Verzeichnis
umgebogen, das Repo bleibt unberuehrt.

    python3 test_env_store.py
"""
import json
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import env_store as es  # noqa: E402
import mesh_utils  # noqa: E402

TRI = [((0, 0, 0), (1, 0, 0), (0, 1, 0))]


def stl(path: Path, z: float = 0.0) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh_utils.write_binary_stl(path, [((0, 0, z), (1, 0, z), (0, 1, z))])
    return path


def env_text(*files, extra="") -> str:
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(files))
    geoms = "".join(f'<geom name="g{i}" type="mesh" mesh="m{i}"/>' for i in range(len(files)))
    return (f"<mujoco>{extra}<asset>{assets}</asset>"
            f"<worldbody><!-- Kommentar bleibt -->{geoms}</worldbody></mujoco>")


def mesh_files(xml: Path) -> list:
    return [m.get("file") for m in ET.parse(xml).getroot().iter("mesh")]


class _Sandbox(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp(prefix="env_store_"))
        self._old = (es.SCENES_DIR, es.MESHES_DIR)
        es.SCENES_DIR = self.root / "scenes"
        es.MESHES_DIR = self.root / "meshes"
        es.SCENES_DIR.mkdir()
        es.MESHES_DIR.mkdir()

    def tearDown(self):
        es.SCENES_DIR, es.MESHES_DIR = self._old
        shutil.rmtree(self.root, ignore_errors=True)

    def make_env(self, name, *meshes) -> Path:
        folder = es.SCENES_DIR / name
        for m in meshes:
            stl(folder / m)
        (folder / es.ENV_FILE).parent.mkdir(parents=True, exist_ok=True)
        (folder / es.ENV_FILE).write_text(env_text(*meshes))
        return folder / es.ENV_FILE

    def cad_group(self, name="zelle", parts=("a.stl", "b.stl"), tag="1") -> Path:
        d = es.MESHES_DIR / "cad" / name
        for i, p in enumerate(parts):
            stl(d / p, z=i)
        (d / es.CAD_MANIFEST).write_text(json.dumps({"tag": tag, "meshes": {}}))
        return d


class TestAuflisten(_Sandbox):
    def test_ordner_alt_und_kaputt(self):
        self.make_env("halle", "meshes/k.stl")
        (es.SCENES_DIR / "alt.xml").write_text("<mujoco/>")
        (es.SCENES_DIR / "MuJoCo Model.xml").write_text("<mujoco/>")
        (es.SCENES_DIR / "leer").mkdir()
        (es.SCENES_DIR / "kaputt.xml").write_text("<nicht")
        good, bad = es.scan()
        self.assertEqual([(e.name, e.legacy) for e in good], [("alt", True), ("halle", False)])
        self.assertEqual(sorted(p.name for p, _ in bad), ["MuJoCo Model.xml", "kaputt.xml", "leer"])

    def test_ordner_gewinnt_gegen_alte_datei(self):
        self.make_env("halle")
        (es.SCENES_DIR / "halle.xml").write_text("<mujoco/>")
        (env,) = es.list_envs()
        self.assertFalse(env.legacy)

    def test_resolve(self):
        xml = self.make_env("halle").resolve()
        for arg in ("halle", "halle.xml", str(es.SCENES_DIR / "halle"), str(xml)):
            self.assertEqual(es.resolve(arg), xml, arg)
        self.assertIsNone(es.resolve("gibtsnicht"))
        self.assertEqual(es.env_name_of(xml), "halle")
        self.assertEqual(es.env_name_of(es.SCENES_DIR / "alt.xml"), "alt")

    def test_namen(self):
        self.assertEqual(es.sanitize_env_name("Küche 2.xml"), "Kueche_2")
        self.assertEqual(es.sanitize_env_name("../x/zelle.zip"), "zelle")
        self.assertEqual(es.sanitize_env_name("  "), "")


class TestEigenstaendig(_Sandbox):
    def test_fremde_dateien_werden_eingesammelt(self):
        kiste = stl(es.MESHES_DIR / "kiste.stl")
        cad = self.cad_group()
        src = self.root / "export.xml"
        src.write_text(env_text(kiste, cad / "a.stl", cad / "b.stl"))
        out = es.make_self_contained(src, es.SCENES_DIR / "neu")
        self.assertEqual(mesh_files(out), ["meshes/kiste.stl", "meshes/zelle/a.stl",
                                           "meshes/zelle/b.stl"])
        self.assertTrue((out.parent / "meshes/zelle" / es.CAD_MANIFEST).is_file())
        self.assertEqual(es.containment_problems(out), [])
        self.assertTrue(kiste.is_file())                          # Bibliothek bleibt
        self.assertIn("Kommentar bleibt", out.read_text())

    def test_gleichnamig_aber_anders_wird_umbenannt(self):
        a = stl(self.root / "x" / "teil.stl", z=0)
        b = stl(self.root / "y" / "teil.stl", z=5)
        c = stl(self.root / "z" / "teil.stl", z=0)                # gleich wie a
        src = self.root / "export.xml"
        src.write_text(env_text(a, b, c))
        out = es.make_self_contained(src, es.SCENES_DIR / "neu")
        self.assertEqual(mesh_files(out), ["meshes/teil.stl", "meshes/teil_2.stl",
                                           "meshes/teil.stl"])

    def test_zwei_verschiedene_cad_importe_gleichen_namens(self):
        one = self.cad_group("zelle", tag="1")
        two = es.MESHES_DIR / "anders" / "zelle"
        for p in ("a.stl",):
            stl(two / p, z=9)
        (two / es.CAD_MANIFEST).write_text(json.dumps({"tag": "2"}))
        src = self.root / "export.xml"
        src.write_text(env_text(one / "a.stl", two / "a.stl"))
        out = es.make_self_contained(src, es.SCENES_DIR / "neu")
        self.assertEqual(mesh_files(out), ["meshes/zelle/a.stl", "meshes/zelle_2/a.stl"])

    def test_meshdir_wird_aufgeloest(self):
        stl(es.MESHES_DIR / "kiste.stl")
        src = es.SCENES_DIR / "alt.xml"
        src.write_text(env_text("kiste.stl", extra='<compiler meshdir="../meshes"/>'))
        out = es.make_self_contained(src, es.SCENES_DIR / "neu")
        self.assertEqual(mesh_files(out), ["meshes/kiste.stl"])
        self.assertIsNone(ET.parse(out).getroot().find("compiler").get("meshdir"))

    def test_prune_entfernt_reste(self):
        xml = self.make_env("halle", "meshes/k.stl", "meshes/weg.stl")
        root = ET.parse(xml).getroot()
        asset = root.find("asset")
        asset.remove(asset.findall("mesh")[1])
        tmp = self.root / "export.xml"
        ET.ElementTree(root).write(tmp)
        es.make_self_contained(tmp, xml.parent, prune=True)
        self.assertFalse((xml.parent / "meshes/weg.stl").exists())
        self.assertTrue((xml.parent / "meshes/k.stl").exists())

    def test_probleme_werden_benannt(self):
        folder = es.SCENES_DIR / "halle"
        folder.mkdir()
        (folder / es.ENV_FILE).write_text(env_text(
            str(stl(self.root / "draussen.stl")), "../fremd.stl", "meshes/fehlt.stl"))
        probs = " | ".join(es.containment_problems(folder / es.ENV_FILE))
        for word in ("absoluter Pfad", "ausserhalb", "fehlt"):
            self.assertIn(word, probs)


class TestMigration(_Sandbox):
    def test_alte_umgebung_wird_ordner(self):
        stl(es.MESHES_DIR / "kiste.stl")
        cad = self.cad_group()
        old = es.SCENES_DIR / "kueche.xml"
        old.write_text(env_text("../meshes/kiste.stl", "../meshes/cad/zelle/a.stl"))
        old.with_suffix(".json").write_text("{}")
        self.assertEqual(es.migrate_all(log=lambda m: None), 1)
        env = es.find("kueche")
        self.assertFalse(env.legacy)
        self.assertEqual(env.xml, es.SCENES_DIR / "kueche" / es.ENV_FILE)
        self.assertEqual(es.containment_problems(env.xml), [])
        self.assertFalse(old.exists())
        self.assertFalse(old.with_suffix(".json").exists())
        self.assertTrue((cad / "a.stl").is_file())                # Quelle bleibt
        self.assertEqual(es.migrate_all(log=lambda m: None), 0)   # idempotent


class TestZip(_Sandbox):
    def test_hin_und_zurueck(self):
        self.make_env("halle", "meshes/k.stl", "meshes/sub/t.stl")
        z = es.pack("halle", self.root / "out")
        self.assertEqual(z.name, "halle.zip")
        with zipfile.ZipFile(z) as zf:
            self.assertEqual(sorted(zf.namelist()), ["halle/meshes/k.stl",
                                                     "halle/meshes/sub/t.stl",
                                                     "halle/umgebung.xml"])
        env = es.unpack(z, name="halle_kopie")
        self.assertEqual(es.containment_problems(env.xml), [])
        with self.assertRaises(es.EnvError):
            es.unpack(z)                                          # halle gibt es schon
        es.unpack(z, force=True)
        self.assertEqual(sorted(es.names()), ["halle", "halle_kopie"])

    def test_zip_ohne_ordner_nimmt_dateinamen(self):
        z = self.root / "Mein Lager.zip"
        with zipfile.ZipFile(z, "w") as zf:
            zf.writestr(es.ENV_FILE, env_text("meshes/k.stl"))
            zf.write(stl(self.root / "k.stl"), "meshes/k.stl")
        self.assertEqual(es.unpack(z).name, "Mein_Lager")

    def test_unsichere_und_kaputte_zips(self):
        bad = self.root / "boese.zip"
        with zipfile.ZipFile(bad, "w") as zf:
            zf.writestr("x/umgebung.xml", "<mujoco/>")
            zf.writestr("x/../../ausbruch.txt", "hallo")
        with self.assertRaises(es.EnvError):
            es.unpack(bad)
        self.assertFalse((self.root / "ausbruch.txt").exists())

        fremd = self.root / "fremd.zip"
        with zipfile.ZipFile(fremd, "w") as zf:
            zf.writestr("x/umgebung.xml", env_text("/abs/kiste.stl"))
        with self.assertRaises(es.EnvError):
            es.unpack(fremd)
        self.assertEqual(es.names(), [])                          # nichts halb entpackt

        leer = self.root / "leer.zip"
        with zipfile.ZipFile(leer, "w") as zf:
            zf.writestr("readme.txt", "x")
        with self.assertRaises(es.EnvError):
            es.unpack(leer)

    def test_pack_verweigert_unfertige_umgebung(self):
        (es.SCENES_DIR / "alt.xml").write_text("<mujoco/>")
        with self.assertRaises(es.EnvError):
            es.pack("alt")


if __name__ == "__main__":
    unittest.main(verbosity=2)
