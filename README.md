# duckietown_new_version — Stack modernizado para Duckiebot DB21-J4

Stack de autonomía **moderno, robusto y extensible** para un Duckiebot **DB21-J4**
(NVIDIA Jetson Nano 4GB), construido **sobre la distro Duckietown `ente`** y
modernizado donde aporta valor:

- **Detección de objetos con YOLO26 → TensorRT** (entrenamiento en Colab/M1, inferencia en la Nana).
- **Seguir línea por visión clásica** (reutiliza `dt-core`, libera la GPU).
- **Planificación de ruta en dos capas**: reactiva (base que siempre funciona) + global cargable.
- **Entorno de entrenamiento reproducible** con **simulación y validación local** (sin robot).

> Diseño completo: [`docs/superpowers/specs/2026-06-16-duckietown-db21j4-modernizado-design.md`](docs/superpowers/specs/2026-06-16-duckietown-db21j4-modernizado-design.md)
> Plan de implementación: [`docs/superpowers/plans/2026-06-16-duckietown-db21j4-modernizado.md`](docs/superpowers/plans/2026-06-16-duckietown-db21j4-modernizado.md)
> Runbook de competencia: [`docs/runbook-competencia.md`](docs/runbook-competencia.md)

## Restricción de hardware (importante)

La Jetson Nano original (P3450) está **End-of-Life**: su software está congelado en
**JetPack 4.6.6 / Ubuntu 18.04 / CUDA 10.2 / Python 3.6**. Por eso:

- El **entrenamiento es siempre fuera del robot** (Colab GPU o MacBook M1 con MPS).
- En el robot la **inferencia es solo TensorRT** (nada de PyTorch/Ultralytics).
- Una sola red en GPU (YOLO26); el carril va por visión clásica en CPU.

## Estructura

```
training/        # Pipeline YOLO26 (Colab + M1): config, dataset, train, export ONNX, validate
perception/      # Paquete ROS: deteccion (TensorRT en robot / ONNX en CPU para simular) + carril clasico
planning/        # Paquete ROS: politica reactiva, grafo de mapa, busqueda de ruta, localizacion, nodos
simulation/      # Simulacion de lazo cerrado del cerebro de planificacion (sin ROS)
bringup/         # Orquestacion: full_stack.launch + docker-compose
tools/           # dts, diagnostico del robot, flasheo SD, build del engine TensorRT
docs/            # Spec, plan, runbook, guia de seguir linea
```

## Quickstart (desarrollo en Mac/Colab)

```bash
make setup          # instala dependencias de desarrollo
make test           # corre toda la suite (40 tests)
make sim            # simula el cerebro de planificacion (sin robot)
```

Otros flujos:

```bash
make dataset-check DATA=datasets/duckietown        # valida la estructura del dataset
make train DATA=datasets/duckietown.yaml           # entrena YOLO26n (M1/MPS)  [o usa el notebook Colab]
make export W=runs/detect/train/weights/best.pt    # exporta a ONNX (opset 12, TensorRT 8.2)
make validate W=best.pt DATA=duckietown.yaml       # reporta mAP50 y si pasa el umbral
```

Notebook de entrenamiento en GPU: [`training/notebooks/colab_yolo26.ipynb`](training/notebooks/colab_yolo26.ipynb).

## Validación local (sin robot)

- **40 tests** (`make test`) cubren: config/dataset, pre/post-proceso de visión (letterbox, escala, filtro),
  política de intersección, grafo + búsqueda de ruta + localización, y la simulación de planificación.
- **`make sim`** valida punta a punta que el planeador global + navegador reactivo llevan al destino.
- La **percepción es simulable en CPU** vía `object_detection.onnx_inference.ONNXDetector`
  (mismo pre/post-proceso que la ruta TensorRT del robot).

## Pasos con hardware (tuyos)

Ver el detalle y los fallbacks en cascada en [`docs/runbook-competencia.md`](docs/runbook-competencia.md):

1. **Preparar el robot**: `./tools/install_dts.sh`, `./tools/check_robot.sh <hostname>`; si hace falta, `./tools/flash_sd.sh <hostname> <ssid> <pass>` (distro `ente`, config DB21J).
2. **Entrenar** YOLO26n (Colab o M1) → exportar a **ONNX**.
3. **Construir el engine** en la Nana: `./tools/build_trt_engine.sh model.onnx model.engine` (copiarlo a `/data/`).
4. **Build y arranque** en el robot: `dts devel build -H <hostname>` y `roslaunch bringup full_stack.launch veh:=<hostname> use_global_planner:=<true|false> use_object_detection:=<true|false>`.
5. **Pendiente de integración on-robot**: conectar `~turn_cmd` del navegador al control de intersecciones de `dt-core`/`dt-car-interface`.

## Extensibilidad

Arquitectura modular para ir incorporando más componentes de Duckietown: AprilTags
(semáforos/señales), freno de emergencia con el ToF VL53L0X, mejor localización con
encoders/IMU, y más pruebas en el simulador Duckiematrix. Cada uno entra como
nodo/módulo nuevo + sus tests, sin tocar lo existente.
