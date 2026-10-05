# Meshes – die Bibliothek

Hier liegen 3D-Objekte (STL/OBJ) und CAD-Dateien (STEP/IGES), aus denen man im
Editor Objekte in eine Umgebung einfuegt. **Umgebungen haengen nicht von diesem
Ordner ab:** beim Speichern kopiert der Editor alles Benutzte in den Ordner der
Umgebung (`scenes/<name>/meshes/`, siehe `../README.md`). Dateien hier duerfen
also umbenannt oder geloescht werden, ohne dass eine Umgebung kaputtgeht.

- `sample_crate.stl`     – 30 cm Wuerfel (Beispiel-Import)
- `sample_ramp.stl`      – kleine Rampe/Keil (Beispiel-Import)
- `sample_bracket.step`  – Winkel-Halterung in mm (Beispiel fuer den CAD-Import)
- `cad/<name>/`          – Zwischenablage fuer CAD-Importe im Editor (nicht im Git)
- `uploads/`             – im Editor hochgeladene CAD-Dateien (nicht im Git)

## Einfuegen im Editor
- **„Eigene Datei hochladen" -> „Datei waehlen (STL/OBJ/STEP/...)"** (oben im
  Editor): Datei-Dialog, beliebiger Ordner.
- **„Add Assets from File" -> „Scan assets" -> „Add asset"**: durchsucht diesen
  Ordner (Einzelteile aus `cad/` werden dort ausgeblendet).
- **„CAD-Import" -> „Importieren & einfuegen"**: STEP/IGES/... aus diesem Ordner.

## CAD (STEP/IGES) und fremde Mesh-Formate
MuJoCo kann nur STL/OBJ/MSH. Alles andere zerlegt `../cad_import.py` in
Einzelteil-Meshes (mit Namen, Farben, Lage; Einheit automatisch in Meter) –
Details im Abschnitt „CAD-Import" in `../README.md`.

## Meshes von Hand in eine Umgebung einbinden
Datei nach `scenes/<name>/meshes/` legen und in `scenes/<name>/umgebung.xml`
relativ referenzieren:

```xml
<asset>
  <mesh name="mein_objekt" file="meshes/mein_objekt.stl"
        scale="0.001 0.001 0.001"/>   <!-- CAD-STL in mm -> Meter -->
</asset>
<worldbody>
  <geom type="mesh" mesh="mein_objekt" pos="1 0 0"/>
</worldbody>
```

`./launch.sh check <name>` prueft, dass die Umgebung eigenstaendig ist.

Tipps:
- MuJoCo laedt nur **binaeres** STL mit hoechstens 200000 Dreiecken.
  ASCII-STL repariert der Editor automatisch; `./launch.sh check-meshes`
  prueft den Bestand.
- **scale** nicht vergessen: STL aus CAD ist oft in Millimetern, MuJoCo
  rechnet in Metern -> `scale="0.001 0.001 0.001"`.
- Fuer **Kollision** sollte das Mesh geschlossen (watertight) und moeglichst
  konvex sein. Komplexe Meshes kollidieren sonst nur mit ihrer konvexen
  Huelle. Reine Deko: `contype="0" conaffinity="0"` am geom setzen.
