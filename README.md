# duckietown_new_version — Stack modernizado para Duckiebot DB21-J4

Stack de autonomía **moderno, robusto y extensible** para un Duckiebot **DB21-J4**
(NVIDIA Jetson Nano 4GB), construido **sobre la distro Duckietown `ente`** y
modernizado donde aporta valor:

- **Detección de objetos con YOLO26 → TensorRT** (entrenamiento en Colab/M1, inferencia en la Nana; mAP50 real **0.937**).
- **Seguir línea por visión clásica** (reutiliza `dt-core`, libera la GPU para una sola red).
- **Planificación de ruta en dos capas**: reactiva (base que siempre funciona) + global A*/BFS cargable.
- **Diagnóstico, calibración, monitoreo de salud por red y validación en un comando**, todo sin depender del shell `dts`.
- **Entorno de entrenamiento reproducible** con **simulación y validación local** (sin robot) — **324 tests**.

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
make                # lista TODOS los comandos (repo + dispositivo) con su ayuda
make setup          # instala dependencias de desarrollo (uv)
make test           # corre toda la suite (324 tests)
make sim            # simula el cerebro de planificacion (sin robot)
make sim-auto       # mision AUTONOMA end-to-end: cerebro completo en lazo cerrado
```

> **Interfaz única:** `make help` (o `make` a secas) lista todo en un solo lugar —
> desarrollo (`setup/test/sim*/train`), dispositivo (`diagnose/sensors/validate-robot/
> calibrate-*/monitor/bypass-info`) e imagen (`build-image/deploy/flash`). Mismos
> comandos en el repo y en el robot.

El **cerebro autónomo integrado** (`duckietown_planning.autonomy.AutonomousController`)
une en un solo controlador: fusión de obstáculos (YOLO26+ToF) → FSM de conducción →
PID de carril → planeador global + política de intersección → localización.
`make sim-auto` lo maneja en **lazo cerrado** (sigue carril, frena ante el pato, para y
gira en intersecciones según la ruta, llega al destino) — todo sin robot.

Entrenamiento / exportación:

```bash
make dataset-check DATA=datasets/duckietown            # valida estructura del dataset
make train DATA=datasets/duckietown.yaml               # entrena YOLO26n (M1/MPS) [o el notebook Colab]
make export W=runs/detect/train/weights/best.pt        # exporta a ONNX (opset 12, TensorRT 8.2)
make validate W=best.pt DATA=duckietown.yaml           # reporta mAP50 y umbral
```

Notebook GPU: [`training/notebooks/colab_yolo26.ipynb`](training/notebooks/colab_yolo26.ipynb).
Resultados: [`docs/resultados-entrenamiento.md`](docs/resultados-entrenamiento.md) (0.833 sintético → 0.871 → **0.937** real; transfer learning +0.273) · [`docs/resultados-rl.md`](docs/resultados-rl.md).

### Manejo manual y entrenamiento por refuerzo (gym)

Ambos **incluidos y testeados**, expuestos en `make`:

```bash
make teleop T=0.6 S=0.2     # manejo MANUAL: mapea throttle/steer -> ruedas (control/teleop.py, 4 tests)
make rl-deps                # instala gymnasium + stable-baselines3
make rl-train STEPS=30000   # entrena PPO de seguir-linea en el gym (rl/, 6 tests)
make rl-eval                # evalua el agente vs la base PD
```

- **Manejo manual:** `control/teleop.py` es el mapeo joystick/teclado→ruedas (la misma
  lógica que alimenta al `joy_mapper` en el robot, encadenable con la calibración).
- **Gym/RL:** `rl/lane_env.py` es un **`gymnasium.Env`** de seguir-línea + PPO
  (`rl/train_ppo.py`) + analítica vs PD. ⚠️ El gym **entrena fuera del robot** (CPU/Mac/
  Colab): la Jetson Nano es EOL y no entrena; la política aprendida se exporta como los
  modelos de visión.

## Validación local (sin robot) — 324 tests

`make test` cubre, con TDD: config/dataset/export de entrenamiento; pre/post-proceso de
visión (letterbox, escala, filtro); multi-cámara y calibración; AprilTags; política de
intersección, grafo+ruta+localización, odometría y safety ToF; PID, teleop y secuencia de
motores; diagnósticos (salud, monitor, I2C, HUT); RL (entorno+analítica); abstracción de placa;
y la simulación de planificación/e2e/percepción. `make sim` valida punta a punta que el
planeador global + navegador reactivo llevan al destino. La **percepción es simulable en CPU**
vía `object_detection.onnx_inference` (mismo pre/post-proceso que la ruta TensorRT del robot).

## En el robot

Mismos comandos `make` (envuelven los `tools/`); ver `make help`.

### Diagnóstico y validación de hardware en un comando

```bash
make diagnose                  # veredicto automatico del bus I2C (stuck_low / controlador) + pasos
make sensors BUS=1             # prueba IMU/ToF/OLED en el bus indicado          [sudo]
make validate-robot            # prueba TODO el hardware SIN mover motores       [sudo]
make validate-robot-motors     # incluye girar motores (¡RUEDAS EN EL AIRE!)     [sudo]
make bypass-info               # cableado de bypass si la HUT esta danada
```

Si la HUT I2C está dañada, `make diagnose` lo dice y `make bypass-info` da el cableado
directo (sensores al header + driver de motor externo @0x60) — ver
[`docs/bypass-hut-cableado-directo.md`](docs/bypass-hut-cableado-directo.md).

Corre con **compuerta de seguridad por el bus I2C**: preflight (escanea PCA9685/IMU/ToF/OLED) →
sensores → cámara → calibración → motores (solo si el bus responde y el PCA9685 está presente).
El test de motores usa el **driver oficial `hat_driver` (HATv3)** dentro del contenedor, con la
secuencia segura de `control/motor_test.py` (PWM ≤ 40%, siempre termina liberando los motores).

### Calibración

```bash
make calibrate-generate HOST=<host> CAMERA=pi-m12                       # base de fabrica
make calibrate-validate CAL=config/calibrations/kinematics/<host>.yaml
```

Genera/valida la base de fábrica (cinemática + intrínsecos). La **calibración fina** (ruedas con
regla, cámara intrínseca y **extrínseca** con tablero 3.1 cm) se hace en el robot — ver el
procedimiento oficial en [`docs/revision-profunda.md`](docs/revision-profunda.md) §3.

### Operación, grabación y monitoreo (por SSH)

```bash
make ssh                       # shell en el robot (var: SSH=prueba@192.168.55.1)
make camera-grab               # captura 1 frame -> ./cam.jpg
make record COUNT=100          # graba 100 frames de camara -> ./dataset/ (para entrenar)
make monitor                   # (en el robot) expone salud OK/DEGRADED/FAULT por HTTP JSON
make health                    # (desde el Mac) consulta ese JSON
make start / make stop / make logs   # ciclo del stack en el robot (docker compose)
```

`make record`/`make camera-grab` usan **gstreamer** en el robot (sin dependencias);
el grabador más completo (salud de frame + `manifest.json`) es
[`tools/record_dataset.py`](tools/record_dataset.py) (lógica pura testeada). CI local
con `make ci`; limpiar con `make clean`.

### Imagen / despliegue / flasheo (sin dts)

```bash
make build-image                                  # imagen arm64 del robot (buildx)
make flash IMG=jetson.img.xz DEV=/dev/rdiskN      # flashea la SD (macOS: rdiskN, Linux: sdX)
make deploy MODE=registry HOST=<host> IMG=<ref>   # despliega y levanta en el robot
```

Detalle y alternativas (engine TRT, `dts`, `roslaunch bringup full_stack.launch`) en
[`docs/runbook-competencia.md`](docs/runbook-competencia.md) y
[`docs/multiplataforma.md`](docs/multiplataforma.md).

## Estado honesto

- **Sólido y testeado (324 tests):** toda la capa algorítmica/CPU (planning, percepción,
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
contenedor dev) · gestor `uv` · CI matriz (Ubuntu+macOS, py3.10-3.13) · transfer learning
validado (+0.273 mAP) · diagnóstico de fallas HW + red · monitoreo de salud EN VIVO por red ·
validación E2E en un comando · soporte de cualquier modelo de cámara (autodetección).

## Licencia

MIT — ver [`LICENSE`](LICENSE). Este stack corre sobre la distro oficial de Duckietown (`ente`) y reutiliza `dt-core` como dependencia; esos componentes conservan su propia licencia.
