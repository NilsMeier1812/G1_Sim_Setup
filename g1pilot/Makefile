# ═══════════════════════════════════════════════════════
# G1Pilot Makefile
#   make build-sim       → Sim-Images bauen (~8 min)
#   make build-real      → Real-Image bauen (schlank, ~10 min, kein Livox/MOLA)
#   make build-real-full → Real-Image MIT Livox/MOLA bauen (~45 min)
#   make sim             → Simulation starten
#   make real            → Echter Roboter starten (schlank)
#   make real-full       → Echter Roboter mit Livox/MOLA/Navigation starten
#   make stop            → Alles stoppen
#   make editor          → Scene Editor im Container (Browser: http://127.0.0.1:8080)
#   make build-editor    → Scene-Editor-Image bauen (~5 min)
#
#   Duenner Wrapper um `docker compose --profile <sim|real|real-full>` auf
#   der EINEN docker-compose.yml (siehe deren Kopfkommentar) -- es gibt
#   bewusst keine separaten docker-compose.*.yaml-Dateien mehr. Einzige
#   Ausnahme: docker-compose.gpu.yml / docker-compose.nvidia.yml (nur GPU-
#   Zugriff fuer die Sim-Container), automatisch dazugenommen, wenn vorhanden.
# ═══════════════════════════════════════════════════════

ROBOT_INTERFACE ?= eth0
DISPLAY         ?= :0
# Argumente fuer launch.sh im Editor-Container (leer = Menue), z.B.
#   make editor ARGS="new"   |   make editor ARGS="import /import/zelle.stp"
ARGS            ?=

# GPU fuer MuJoCo-Viewer/RViz (sonst CPU-Rendering): /dev/dri -> docker-compose.gpu.yml,
# NVIDIA + Container Toolkit -> docker-compose.nvidia.yml. Wie in start.sh;
# per COMPOSE_FILE uebersteuerbar.
HAS_NVIDIA := $(shell nvidia-smi -L >/dev/null 2>&1 && command -v nvidia-container-runtime-hook >/dev/null 2>&1 && echo 1)
COMPOSE_FILE ?= docker-compose.yml$(if $(wildcard /dev/dri),:docker-compose.gpu.yml)$(if $(HAS_NVIDIA),:docker-compose.nvidia.yml)
export COMPOSE_FILE
# PRIME Render Offload nur, wenn die NVIDIA KEINEN Bildschirm treibt (Hybrid-
# Laptop); sonst bleibt das MuJoCo-Fenster schwarz. Wie in start.sh.
G1_NV_PRIME_OFFLOAD ?= $(if $(HAS_NVIDIA),$(if $(shell nvidia-smi --query-gpu=display_active --format=csv,noheader 2>/dev/null | grep Enabled),0,1),0)
export G1_NV_PRIME_OFFLOAD

.PHONY: sim sim-bg real real-full stop \
        build-sim build-real build-real-full \
        build-mujoco build-g1pilot-sim build-g1pilot-real build-g1pilot-real-full \
        logs status clean shell-sim shell-mujoco shell-real \
        editor build-editor

# ─── SIMULATION ─────────────────────────────────────────────
sim: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte G1Pilot SIMULATION..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose --profile sim up

sim-bg: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte SIMULATION (Hintergrund)..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose --profile sim up -d

# ─── ECHTER ROBOTER ─────────────────────────────────────────
real: build-g1pilot-real
	@echo "🔴 Starte G1Pilot ECHTER ROBOTER (schlank: Arme + Haende + Loco)..."
	@xhost +local:root 2>/dev/null || true
	ROBOT_INTERFACE=$(ROBOT_INTERFACE) DISPLAY=$(DISPLAY) \
	  docker compose --profile real up

real-full: build-g1pilot-real-full
	@echo "🔴 Starte G1Pilot ECHTER ROBOTER (voll: + Livox/MOLA/Navigation)..."
	@xhost +local:root 2>/dev/null || true
	ROBOT_INTERFACE=$(ROBOT_INTERFACE) DISPLAY=$(DISPLAY) \
	  docker compose --profile real-full up

# ─── SCENE EDITOR (Container, v.a. fuer Windows) ────────────
# `run` statt `up`: launch.sh ist interaktiv (Menue) und braucht ein Terminal.
editor: build-editor
	@echo "🟢 Scene Editor: http://127.0.0.1:8080 (Strg+C beendet)"
	HOST_UID=$$(id -u 2>/dev/null || echo 1000) HOST_GID=$$(id -g 2>/dev/null || echo 1000) \
	  docker compose --profile editor run --rm --service-ports scene-editor bash launch.sh $(ARGS)

build-editor:
	@echo "📦 Baue Scene-Editor Image (~5 min)..."
	docker compose --profile editor build scene-editor

# ─── BAUEN ──────────────────────────────────────────────────
build-sim: build-g1pilot-sim build-mujoco

build-real: build-g1pilot-real

build-real-full: build-g1pilot-real-full

build-g1pilot-sim:
	@echo "📦 Baue G1Pilot SIM Image (kein Livox/MOLA, ~8 min)..."
	docker build -f docker/Dockerfile.sim -t g1pilot-sim:v1.1.0 .

build-g1pilot-real:
	@echo "📦 Baue G1Pilot REAL Image (schlank, ~10 min)..."
	docker build -f docker/Dockerfile.real -t g1pilot-real:v1.2.0 .

build-g1pilot-real-full:
	@echo "📦 Baue G1Pilot REAL-FULL Image (+ Livox/MOLA, ~45 min)..."
	docker build -f docker/Dockerfile -t g1pilot:v1.1.0 .

build-mujoco:
	@echo "📦 Baue MuJoCo Simulator Image..."
	docker build -f docker/Dockerfile.mujoco -t g1pilot-mujoco:v1.0 ..

# ─── STOPPEN ────────────────────────────────────────────────
stop:
	docker compose --profile sim down --remove-orphans 2>/dev/null || true
	docker compose --profile real down --remove-orphans 2>/dev/null || true
	docker compose --profile real-full down --remove-orphans 2>/dev/null || true
	docker compose --profile editor down --remove-orphans 2>/dev/null || true

# ─── HILFS-COMMANDS ─────────────────────────────────────────
logs:
	docker compose --profile sim logs -f

status:
	docker compose --profile sim --profile real --profile real-full ps

clean:
	docker compose --profile sim down --volumes --remove-orphans 2>/dev/null || true
	docker compose --profile real down --volumes --remove-orphans 2>/dev/null || true
	docker compose --profile real-full down --volumes --remove-orphans 2>/dev/null || true
	docker compose --profile editor down --volumes --remove-orphans 2>/dev/null || true
	docker rmi g1pilot-sim:v1.1.0 g1pilot-mujoco:v1.0 g1pilot-real:v1.2.0 g1pilot:v1.1.0 \
	  g1pilot-scene-editor:v1.0 2>/dev/null || true

# Shells ueber `docker compose exec` statt Name-Filter -- Compose ohne
# `container_name:` benennt Container nach <projekt>-<service>-<n>, das
# haengt vom Verzeichnisnamen/-COMPOSE_PROJECT_NAME ab.
shell-sim:
	docker compose --profile sim exec g1pilot-sim bash

shell-mujoco:
	docker compose --profile sim exec mujoco-sim bash

shell-real:
	docker compose --profile real exec g1pilot-real bash
