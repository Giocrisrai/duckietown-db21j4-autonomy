# Makefile — interfaz unica del stack DB21-J4 (repo + dispositivo).
# `make` o `make help` lista todo. Variables: BUS=1 HOST=prueba CAMERA=pi-m12
#   IMG=<imagen> DEV=<disco> MODE=registry|tar CAL=<yaml>
.PHONY: help setup test sim sim-auto sim-loop sim-mission tile-route \
        dataset-check train export validate teleop \
        rl-deps rl-train rl-eval \
        diagnose sensors validate-robot validate-robot-motors \
        calibrate-generate calibrate-validate monitor bypass-info \
        build-image deploy flash

BUS    ?= 1
HOST   ?= prueba
CAMERA ?= pi-m12
MODE   ?= registry
STEPS  ?= 30000
T      ?= 0.6
S      ?= 0.2

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
	sudo python3 tools/test_sensors.py $(BUS)

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

# ───────────────────────── Imagen / despliegue / flasheo ──────────────────────────
build-image:     ## construye la imagen arm64 del robot (buildx, sin dts)
	bash tools/build_image.sh

deploy:          ## despliega al robot (uso: make deploy MODE=registry|tar HOST=prueba IMG=<ref>)
	bash tools/deploy_robot.sh $(MODE) $(HOST) $(IMG)

flash:           ## flashea la SD (uso: make flash IMG=jetson.img.xz DEV=/dev/rdiskN)
	bash tools/flash_sd_raw.sh $(IMG) $(DEV)
