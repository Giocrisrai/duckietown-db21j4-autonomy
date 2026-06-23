# Makefile — interfaz unica del stack DB21-J4 (repo + dispositivo).
# `make` o `make help` lista todo. Variables: BUS=1 HOST=prueba CAMERA=pi-m12
#   IMG=<imagen> DEV=<disco> MODE=registry|tar CAL=<yaml>
.PHONY: help setup test ci clean lint sim sim-auto sim-loop sim-mission tile-route \
        dataset-check train export validate teleop \
        rl-deps rl-train rl-eval \
        diagnose sensors validate-robot validate-robot-motors \
        calibrate-generate calibrate-validate monitor bypass-info \
        ssh camera-grab record health start stop logs \
        build-image deploy flash

BUS    ?= 1
HOST   ?= prueba
CAMERA ?= pi-m12
MODE   ?= registry
STEPS  ?= 30000
T      ?= 0.6
S      ?= 0.2
# endpoint SSH del robot por USB (override: make ssh SSH=duckie@robot.local)
SSH     ?= prueba@192.168.55.1
COUNT   ?= 50
# ruta del compose en el robot (la deja `make deploy`) y puerto del monitor
COMPOSE ?= /tmp/dt-compose.yaml
PORT    ?= 8080

PYDEMO = PYTHONPATH=simulation:planning/packages/duckietown_planning/src:perception/packages/object_detection/src:control

help:            ## muestra esta ayuda (objetivo por defecto)
	@echo "Uso: make <objetivo>   (vars: BUS HOST CAMERA IMG DEV MODE CAL)"
	@echo ""
	@grep -hE '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-24s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help

# ───────────────────────── Repo / desarrollo (sin robot) ─────────────────────────
setup:           ## instala dependencias de desarrollo (Mac/Colab)
	pip install -r requirements-dev.txt

test:            ## corre toda la suite de tests
	python -m pytest -q

lint:            ## linter (ruff): caza nombres indefinidos, imports/vars sin usar
	python -m ruff check .

ci:              ## corre lo mismo que CI (lint + suite completa)
	python -m ruff check . && python -m pytest -q

clean:           ## borra caches y artefactos (__pycache__, .pytest_cache, *.pyc)
	find . -path ./.venv -prune -o -name __pycache__ -exec rm -rf {} + 2>/dev/null; \
	  rm -rf .pytest_cache; echo "limpio"

sim:             ## simulacion de lazo cerrado del planeador
	python -m pytest simulation/tests -q

sim-auto:        ## demo: mision autonoma end-to-end (cerebro completo, sin robot)
	$(PYDEMO) python -m sim_autonomy

sim-loop:        ## demo: lazo cerrado imagen->imagen (el cerebro solo ve pixeles)
	$(PYDEMO) python -m sim_closed_loop

sim-mission:     ## demo: mision completa multi-interseccion en lazo imagen->imagen
	$(PYDEMO) python -m sim_mission

tile-route:      ## demo: navegacion tile-to-tile sobre un mapa de tiles completo
	PYTHONPATH=planning/packages/duckietown_planning/src python -m duckietown_planning.tile_map

teleop:          ## manejo MANUAL: mapea throttle/steer a ruedas (uso: make teleop T=0.6 S=0.2)
	PYTHONPATH=control python -m teleop $(T) $(S)

# ───────────────────────── RL / gym (entrena FUERA del robot) ─────────────────────
rl-deps:         ## instala dependencias de RL (gymnasium + stable-baselines3)
	pip install -r requirements-rl.txt

rl-train:        ## entrena PPO de seguir-linea en el gym (uso: make rl-train STEPS=30000)
	python -m rl.train_ppo --timesteps $(STEPS)

rl-eval:         ## evalua PPO vs base PD en el gym (uso: make rl-eval STEPS=5000)
	python -m rl.train_ppo --timesteps $(STEPS) --episodes-eval 20

dataset-check:   ## valida el dataset (uso: make dataset-check DATA=datasets/duckietown)
	PYTHONPATH=training python -c "from yolo26.dataset_check import check_dataset; print(check_dataset('$(DATA)'))"

train:           ## entrena YOLO26n (uso: make train DATA=datasets/duckietown.yaml)
	cd training && python -m yolo26.train --data $(DATA)

export:          ## exporta a ONNX (uso: make export W=runs/detect/train/weights/best.pt)
	cd training && python -c "from yolo26.export_onnx import export; print(export('$(W)'))"

validate:        ## valida el modelo (uso: make validate W=best.pt DATA=duckietown.yaml)
	cd training && python -c "from yolo26.validate import validate, passes_threshold; m=validate('$(W)','$(DATA)'); print('mAP50=', m, 'OK' if passes_threshold(m) else 'BAJO')"

# ───────────────────────── Dispositivo (en el robot / Jetson) ─────────────────────
diagnose:        ## diagnostico I2C automatico (veredicto + pasos)  [sudo]
	sudo python3 tools/diagnose_i2c.py

sensors:         ## prueba sensores I2C del bus BUS (IMU/ToF/OLED)   [sudo]
	sudo python3 tools/detect_sensors.py $(BUS)

validate-robot:  ## valida TODO el hardware SIN mover motores        [sudo]
	sudo bash tools/validate_robot.sh

validate-robot-motors: ## valida hardware girando motores (¡RUEDAS EN EL AIRE!)  [sudo]
	sudo bash tools/validate_robot.sh --motors

calibrate-generate:    ## genera calibracion base (uso: make calibrate-generate HOST=prueba CAMERA=pi-m12)
	python3 tools/calibrate.py generate --robot $(HOST) --camera $(CAMERA)

calibrate-validate:    ## valida un YAML de calibracion (uso: make calibrate-validate CAL=config/.../prueba.yaml)
	python3 tools/calibrate.py validate $(CAL)

monitor:         ## monitor de salud por red (HTTP JSON OK/DEGRADED/FAULT)
	python3 diagnostics/run_monitor.py

bypass-info:     ## imprime el cableado de bypass de la HUT (sensores directos + driver externo)
	python3 -c "import sys; sys.path.insert(0,'diagnostics'); import wiring; print(wiring.wiring_summary())"

# ───────────────────────── Conexion / captura / grabacion (por SSH) ────────────────
ssh:             ## abre una shell en el robot (var: SSH=prueba@192.168.55.1)
	ssh $(SSH)

camera-grab:     ## captura 1 frame de la camara del robot y lo baja a ./cam.jpg
	ssh $(SSH) 'timeout 12 gst-launch-1.0 -e nvarguscamerasrc num-buffers=1 ! "video/x-raw(memory:NVMM),width=1280,height=720" ! nvjpegenc ! filesink location=/tmp/cam.jpg' >/dev/null 2>&1
	scp $(SSH):/tmp/cam.jpg ./cam.jpg && echo "guardado ./cam.jpg"

record:          ## graba COUNT frames de la camara a ./dataset/ (uso: make record COUNT=100)
	ssh $(SSH) 'rm -rf /tmp/ds && mkdir -p /tmp/ds && timeout 60 gst-launch-1.0 -e nvarguscamerasrc num-buffers=$(COUNT) ! "video/x-raw(memory:NVMM),width=1280,height=720,framerate=10/1" ! nvjpegenc ! multifilesink location=/tmp/ds/frame_%05d.jpg' >/dev/null 2>&1
	mkdir -p dataset && scp -r $(SSH):/tmp/ds/. ./dataset/ && echo "guardados en ./dataset/ ($(COUNT) frames)"

health:          ## consulta el monitor de salud del robot (JSON; var: PORT=8080)
	curl -s http://$(word 2,$(subst @, ,$(SSH))):$(PORT)/health || echo "(monitor no responde; arrancalo con: make monitor en el robot)"

# ───────────────────────── Ciclo del stack en el robot (docker compose) ────────────
start:           ## arranca el stack en el robot (docker compose up -d)  [sudo en robot]
	ssh $(SSH) 'VEHICLE_NAME=$$(hostname) docker compose -f $(COMPOSE) up -d' && echo "stack arriba"

stop:            ## detiene el stack en el robot (docker compose down)
	ssh $(SSH) 'docker compose -f $(COMPOSE) down' && echo "stack detenido"

logs:            ## sigue los logs del stack en el robot
	ssh $(SSH) 'docker compose -f $(COMPOSE) logs -f'

# ───────────────────────── Imagen / despliegue / flasheo ──────────────────────────
build-image:     ## construye la imagen arm64 del robot (buildx, sin dts)
	bash tools/build_image.sh

deploy:          ## despliega al robot (uso: make deploy MODE=registry|tar HOST=prueba IMG=<ref>)
	bash tools/deploy_robot.sh $(MODE) $(HOST) $(IMG)

flash:           ## flashea la SD (uso: make flash IMG=jetson.img.xz DEV=/dev/rdiskN)
	bash tools/flash_sd_raw.sh $(IMG) $(DEV)
