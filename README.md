# duckietown_new_version — Stack modernizado para Duckiebot DB21-J4

Stack de autonomía **moderno, robusto y extensible** para un Duckiebot **DB21-J4**
(NVIDIA Jetson Nano 4GB), construido **sobre la distro Duckietown `ente`** y
modernizado donde aporta valor:

- **Detección de objetos con YOLO26 → TensorRT** (entrenamiento en Colab/M1, inferencia en la Nana; mAP50 real **0.937**).
- **Seguir línea por visión clásica** (reutiliza `dt-core`, libera la GPU para una sola red).
- **Planificación de ruta en dos capas**: reactiva (base que siempre funciona) + global A*/BFS cargable.
- **Diagnóstico, calibración, monitoreo de salud por red y validación en un comando**, todo sin depender del shell `dts`.
- **Entorno de entrenamiento reproducible** con **simulación y validación local** (sin robot) — **173 tests**.

> **Revisión profunda y brechas vs Duckietown oficial:** [`docs/revision-profunda.md`](docs/revision-profunda.md) ← empieza por aquí para el panorama completo.
>
> Diseño: [`spec`](docs/superpowers/specs/2026-06-16-duckietown-db21j4-modernizado-design.md) ·
> Plan: [`plan`](docs/superpowers/plans/2026-06-16-duckietown-db21j4-modernizado.md) ·
> Runbook competencia: [`docs/runbook-competencia.md`](docs/runbook-competencia.md) ·
> [Checklist de paridad](docs/checklist-paridad.md)

## Restricción de hardware (importante)

La Jetson Nano original (P3450) está **End-of-Life**: su software está congelado en
**JetPack 4.6.6 / Ubuntu 18.04 / CUDA 10.2 / Python 3.6**. Por eso:

- El **entrenamiento es siempre fuera del robot** (Colab GPU o MacBook M1 con MPS).
- En el robot la **inferencia es solo TensorRT** (nada de PyTorch/Ultralytics).
- Una sola red en GPU (YOLO26); el carril va por visión clásica en CPU.
- Las herramientas son compatibles con **Python 3.6** del robot (ver `tools/`).

## Estructura

```
training/         # Pipeline YOLO26 (Colab + M1): config, dataset, train, export ONNX, validate
perception/       # ROS: deteccion (TensorRT robot / ONNX CPU para simular), multi-camara,
                  #      calibracion de camara, salud de camara, AprilTags, seleccion de backend
planning/         # ROS: interseccion reactiva, grafo+A*/BFS, localizacion, odometria, safety ToF,
                  #      calibracion cinematica, nodos (freno, navegador, planeador)
control/          # PID + sim de carril, teleop, secuencia segura de motores (logica pura)
simulation/       # Lazo cerrado del cerebro (planning, e2e, percepcion ONNX, apriltags) sin ROS
diagnostics/      # Salud (bateria/encoders/IMU/ToF), monitor con debounce, servidor HTTP, I2C, HUT
rl/               # Entorno Gymnasium lane-following + PPO + analitica/baselines
platform_support/ # Abstraccion de placa (Jetson Nano/Orin/RaspberryPi/x86) -> backend + camara
tools/            # Diagnostico del robot, flasheo SD, build TRT, calibracion, validacion en 1 comando
bringup/          # Orquestacion: launch + docker-compose
config/           # Calibraciones de fabrica (kinematics + camera_intrinsic)
docs/             # Revision profunda, spec, plan, runbook, calibracion, hardware, resultados
```

## Quickstart (desarrollo en Mac/Colab)

```bash
make setup          # instala dependencias de desarrollo (uv)
make test           # corre toda la suite (173 tests)
make sim            # simula el cerebro de planificacion (sin robot)
```

Entrenamiento / exportación:

```bash
make dataset-check DATA=datasets/duckietown            # valida estructura del dataset
make train DATA=datasets/duckietown.yaml               # entrena YOLO26n (M1/MPS) [o el notebook Colab]
make export W=runs/detect/train/weights/best.pt        # exporta a ONNX (opset 12, TensorRT 8.2)
make validate W=best.pt DATA=duckietown.yaml           # reporta mAP50 y umbral
```

Notebook GPU: [`training/notebooks/colab_yolo26.ipynb`](training/notebooks/colab_yolo26.ipynb).
Resultados: [`docs/resultados-entrenamiento.md`](docs/resultados-entrenamiento.md) (0.833 sintético → 0.871 → **0.937** real; transfer learning +0.273) · [`docs/resultados-rl.md`](docs/resultados-rl.md).

## Validación local (sin robot) — 173 tests

`make test` cubre, con TDD: config/dataset/export de entrenamiento; pre/post-proceso de
visión (letterbox, escala, filtro); multi-cámara y calibración; AprilTags; política de
intersección, grafo+ruta+localización, odometría y safety ToF; PID, teleop y secuencia de
motores; diagnósticos (salud, monitor, I2C, HUT); RL (entorno+analítica); abstracción de placa;
y la simulación de planificación/e2e/percepción. `make sim` valida punta a punta que el
planeador global + navegador reactivo llevan al destino. La **percepción es simulable en CPU**
vía `object_detection.onnx_inference` (mismo pre/post-proceso que la ruta TensorRT del robot).

## En el robot

### Validación de hardware en un comando

```bash
sudo bash tools/validate_robot.sh            # prueba todo SIN mover motores
sudo bash tools/validate_robot.sh --motors   # incluye girar motores (¡ruedas en el aire!)
```

Corre con **compuerta de seguridad por el bus I2C**: preflight (escanea PCA9685/IMU/ToF/OLED) →
sensores → cámara → calibración → motores (solo si el bus responde y el PCA9685 está presente).
El test de motores usa el **driver oficial `hat_driver` (HATv3)** dentro del contenedor, con la
secuencia segura de `control/motor_test.py` (PWM ≤ 40%, siempre termina liberando los motores).

### Calibración

```bash
python3 tools/calibrate.py generate --robot <host> --camera pi-m12   # base de fabrica
python3 tools/calibrate.py validate config/calibrations/kinematics/<host>.yaml
```

Genera/valida la base de fábrica (cinemática + intrínsecos). La **calibración fina** (ruedas con
regla, cámara intrínseca y **extrínseca** con tablero 3.1 cm) se hace en el robot — ver el
procedimiento oficial en [`docs/revision-profunda.md`](docs/revision-profunda.md) §3.

### Monitoreo de salud por red

```bash
python3 diagnostics/run_monitor.py     # expone estado OK/DEGRADED/FAULT por HTTP JSON
```

### Despliegue / arranque

Ver [`docs/runbook-competencia.md`](docs/runbook-competencia.md) y [`docs/multiplataforma.md`](docs/multiplataforma.md):
preparar el robot (`tools/install_dts.sh`, `tools/check_robot.sh`), flashear (`tools/flash_sd.sh`,
distro `ente`), construir el engine TRT (`tools/build_trt_engine.sh`), `dts devel build` y
`roslaunch bringup full_stack.launch`.

## Estado honesto

- **Sólido y testeado (173 tests):** toda la capa algorítmica/CPU (planning, percepción,
  training, control, diagnósticos, RL, simulación).
- **Implementado pero NO verificado en hardware:** los nodos ROS, el engine TensorRT real, el
  envío de motores y la calibración fina. El lazo de marcha se apoya en el `lane_controller`
  externo de `dt-core`.
- **Robot `prueba`:** fallo físico de hardware (bus I2C-1 caído, cámara CSI con falla de
  contacto) — ver [`docs/informe-hardware-prueba.md`](docs/informe-hardware-prueba.md) y
  [`docs/diagnostico-hardware-campo.md`](docs/diagnostico-hardware-campo.md).
- **Brechas funcionales vs oficial** (línea roja de stop, ground projection + filtro d/φ,
  anti-Instagram, FSM, fusión visual+ToF, calibración extrínseca): priorizadas con roadmap en
  [`docs/revision-profunda.md`](docs/revision-profunda.md).

## Plus sobre el stack oficial

Independencia de `dts` (build/deploy/flash propios) · multiplataforma (cualquier Linux/macOS +
contenedor dev) · gestor `uv` · CI matriz (Ubuntu+macOS, py3.11/3.12) · transfer learning
validado (+0.273 mAP) · diagnóstico de fallas HW + red · monitoreo de salud EN VIVO por red ·
validación E2E en un comando · soporte de cualquier modelo de cámara (autodetección).
