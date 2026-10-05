#!/usr/bin/env bash
# =====================================================================
# Einmaliges Setup fuer den mujoco-scene-editor.
# Legt ein eigenes virtualenv (.venv) an und installiert alles darin,
# damit die Systeminstallation nicht angefasst wird.
#
#   ./setup.sh
#
# Danach: ./launch.sh edit
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")"

# Ort des venv: Default .venv/ hier im Ordner. Das Docker-Image
# (g1pilot/docker/Dockerfile.scene_editor) legt es nach /opt, damit es nicht im
# gemounteten Repo landet.
VENV="${SCENE_EDITOR_VENV:-.venv}"

# mujoco-scene-editor 0.1.3 verlangt Python >=3.10,<3.13. Ubuntu 26.04 bringt
# nur 3.14 mit -> passenden Interpreter suchen: $PYTHON, python3.12..3.10,
# ein per uv installiertes 3.12, zuletzt python3.
py_ok() { "$1" -c 'import sys; sys.exit(not ((3, 10) <= sys.version_info[:2] < (3, 13)))' 2>/dev/null; }

UV="$(command -v uv || echo "$HOME/.local/bin/uv")"
UV_PY=""
[[ -x "$UV" ]] && UV_PY="$("$UV" python find 3.12 2>/dev/null || true)"

PY=""
for cand in ${PYTHON:-} python3.12 python3.11 python3.10 "$UV_PY" python3; do
  [[ -n "$cand" ]] && command -v "$cand" >/dev/null 2>&1 && py_ok "$cand" && { PY="$cand"; break; }
done
if [[ -z "$PY" ]]; then
  echo "FEHLER: kein Python 3.10-3.12 gefunden (mujoco-scene-editor laeuft nicht" >&2
  echo "        mit $(python3 --version 2>&1)). Ohne sudo z.B. per uv:" >&2
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  echo "  ~/.local/bin/uv python install 3.12" >&2
  echo "  ./setup.sh" >&2
  echo "Oder den Editor im Container nutzen:  cd g1pilot && make editor" >&2
  exit 1
fi
echo ">> Nutze $PY ($("$PY" --version 2>&1))"

# Ohne ensurepip (Debian/Ubuntu: Paket python3-venv) legt 'venv' ein .venv
# OHNE pip an und bricht ab -> kaputtes venv, das launch.sh spaeter findet.
if ! "$PY" -c "import ensurepip" 2>/dev/null; then
  echo "FEHLER: venv-Modul fehlt fuer $PY. Bitte installieren:" >&2
  echo "  sudo apt install -y python3-venv" >&2
  exit 1
fi

# Reste eines abgebrochenen Setups (venv ohne pip) oder ein venv mit falscher
# Python-Version entfernen.
if [[ -d "$VENV" ]] && { [[ ! -x "$VENV/bin/pip" ]] || ! py_ok "$VENV/bin/python"; }; then
  echo ">> Unvollstaendiges/inkompatibles $VENV gefunden -> wird neu angelegt."
  rm -rf "$VENV"
fi

echo ">> Erstelle virtualenv in $VENV ..."
"$PY" -m venv "$VENV"

# pip/setuptools/wheel aktualisieren.
# WICHTIG: ohne aktuelles setuptools scheitert der Build der Abhaengigkeit
# 'GPUtil' (AttributeError: install_layout) mit dem alten System-setuptools.
echo ">> Aktualisiere pip / setuptools / wheel ..."
"$VENV/bin/pip" install --upgrade pip setuptools wheel

echo ">> Installiere mujoco-scene-editor (+ yourdfpy, + CAD-Import) ... das dauert einen Moment."
"$VENV/bin/pip" install -r requirements.txt

# CAD-Import pruefen (cadquery-ocp ist ein grosses Wheel; wenn es fehlt, soll
# das hier auffallen und nicht erst beim Hochladen einer CAD-Datei).
if "$VENV/bin/python" cad_import.py --check >/dev/null 2>&1; then
  echo ">> CAD-Import (STEP/IGES) ist aktiv."
else
  echo ">> WARNUNG: CAD-Import inaktiv (cadquery-ocp fehlt)."
  echo "   Nachinstallieren:  $VENV/bin/pip install cadquery-ocp coacd"
fi

# Persistente RoBits-Config (sonst meckert der Editor und nutzt /tmp)
mkdir -p ".robits_config"

echo ""
echo "============================================================"
echo " Setup fertig."
echo ""
echo " Naechste Schritte:"
echo "   ./launch.sh edit     # Beispiel-Umgebung im Browser bearbeiten"
echo "   ./launch.sh new      # leere Szene starten"
echo "   ./launch.sh view-g1  # G1 + Objekte im MuJoCo-Viewer ansehen"
echo ""
echo " Hinweis: Beim allerersten Start laedt der Editor einmalig den"
echo " Objaverse-Objektkatalog aus dem Netz (braucht Internet, wird"
echo " danach gecacht)."
echo "============================================================"
