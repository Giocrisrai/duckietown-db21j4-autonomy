# Runbook de competencia — DB21-J4

## Antes (en casa)
- [ ] `python -m pytest -v` todo verde.
- [ ] Engine `model.engine` construido y copiado a `/data/` del robot.
- [ ] Vuelta completa de seguir línea OK (Task 12).
- [ ] Detección de patos OK a ~10–25 FPS (Task 11).
- [ ] Intersecciones reactivas OK (Task 14).
- [ ] Batería cargada; SD respaldada.

## En el venue
1. Encender robot, `tools/check_robot.sh <hostname>` → `[OK]`.
2. Ajustar HSV a la luz del lugar si el carril falla (docs/lane-following.md).
3. Si conoces el trazado: armar `planning/maps/<pista>.yaml` y lanzar con
   `use_global_planner:=true start:=<...> goal:=<...>`.
4. Si NO lo conoces: dejar `use_global_planner:=false` (reactivo).

## Fallbacks en cascada (de más a menos funcionalidad)
- Falla el planeador global → `use_global_planner:=false` (reactivo siempre funciona).
- Falla YOLO/TensorRT → arrancar con `use_object_detection:=false` (solo seguir línea + intersecciones).
- Falla una intersección → bajar velocidad (`lane_controller` `v_bar`) y reintentar.
- Todo lo demás falla → `lane_following.launch` solo (mínimo: dar vueltas al loop).
- NOTA: el comando de giro (~turn_cmd del navegador) debe conectarse al control de intersecciones de dt-core/dt-car-interface en el robot para que el giro se ejecute físicamente. Verificar este puente en el primer arranque.

## Entorno de entrenamiento (Colab / M1)
- `make setup` — instala dependencias de desarrollo.
- `make test` — corre toda la suite (debe quedar verde antes de entrenar).
- `make sim` — simula el cerebro de planificación sin robot.
- `make dataset-check DATA=datasets/duckietown` — valida el dataset.
- Entrenar: `make train DATA=datasets/duckietown.yaml` (M1/MPS) o el notebook `training/notebooks/colab_yolo26.ipynb` (Colab GPU).
- `make export W=runs/detect/train/weights/best.pt` — exporta a ONNX.
- `make validate W=... DATA=...` — reporta mAP50 y si pasa el umbral.
- Llevar el `.onnx` al robot y construir el engine con `tools/build_trt_engine.sh`.

## Build y despliegue en el robot
1. `dts devel build -H <hostname>` — construye la imagen arm64 con nuestros paquetes (usa el Dockerfile).
2. Copiar el engine: dejar `model.engine` en `/data/` del robot (ver build_trt_engine.sh).
3. `dts devel run -H <hostname>` o `roslaunch bringup full_stack.launch veh:=<hostname>`.
NOTA: si `dts devel build` reporta un base image distinto, ajustar BASE_IMAGE/BASE_TAG y TYPE_VERSION del .dtproject al template-ros de tu version de dts.

## Comando de arranque
roslaunch bringup full_stack.launch veh:=<hostname> use_global_planner:=<true|false> use_object_detection:=<true|false>
