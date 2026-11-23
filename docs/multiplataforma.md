# Multiplataforma y sin dependencia de `dts`

El proyecto se configura, valida y simula en **cualquier Linux o macOS**, y se
despliega a la Jetson **sin depender del `dts` shell** de Duckietown — usamos
herramientas estándar (Docker buildx, ssh, dd/balena) que podemos tocar, mejorar
y probar nosotros. `dts` queda **opcional**.

## 1. Entorno de desarrollo (cualquier SO)

Usamos **`uv`** (Astral), el gestor de paquetes moderno (10-100× más rápido que pip).
`setup_env.sh` lo usa si está disponible, con fallback automático a venv+pip.

```bash
tools/setup_env.sh          # detecta SO/arch, crea .venv (uv si hay), instala deps, valida
tools/validate_stack.sh     # valida TODO: tests + launches + YAML + scripts + compose
```

**Dependencias:**
- `requirements-dev.txt` — núcleo, todo con wheels (numpy, opencv-headless, onnx,
  onnxruntime, gymnasium, pytest). Construye en cualquier entorno sin compilar ni apt.
- `requirements-apriltag.txt` — opcional (extensiones nativas para detectar AprilTags);
  los tests de AprilTag se **saltan** si no está instalado.
- `requirements-rl.txt` — Deep RL (stable-baselines3). `training/requirements.txt` — YOLO.

O containerizado (host-independiente, no necesita Python local):

```bash
docker compose -f docker-compose.dev.yaml up --build     # corre la suite en contenedor
docker compose -f docker-compose.dev.yaml run dev bash   # shell interactivo
```

## 2. Validar y simular ANTES de tocar el dispositivo

Todo se valida/simula sin robot:
- `tools/validate_stack.sh` — 393 tests + configs + compose.
- `make sim` — simulación de planificación (lazo cerrado).
- Percepción ONNX, AprilTags, PID y RL: tienen simulación/validación propia.

## 3. Construir la imagen del robot (arm64) SIN dts

```bash
tools/build_image.sh                                   # build arm64 local (buildx)
tools/build_image.sh duckietown-new-version:latest --save robot.tar   # tarball portatil
tools/build_image.sh tag --push <registry>/duckietown-new-version     # a un registry
```

Cross-build desde x86 o Mac arm vía emulación QEMU de buildx. No requiere `dts devel build`.

## 4. Desplegar y levantar contenedores en la Jetson SIN dts

```bash
# opcion A: via registry
tools/deploy_robot.sh registry <hostname> <registry>/duckietown-new-version
# opcion B: via tarball (scp + docker load)
tools/deploy_robot.sh tar <hostname> robot.tar
```

Esto copia `bringup/docker-compose.yaml` y hace `docker compose up -d` en el robot.
Para correr a mano en el robot: `docker compose -f docker-compose.yaml up`.

## 5. Flashear la SD SIN dts (multiplataforma)

```bash
tools/flash_sd_raw.sh <imagen-jetson.img.xz> <dispositivo>
#   macOS:  /dev/rdiskN  (diskutil list)
#   Linux:  /dev/sdX     (lsblk)
```

Usa balena CLI si está, o `dd` (bs ajustado por SO). La imagen base puede ser
JetPack 4.6.x o una imagen comunitaria; el stack corre en contenedores encima.

## Matriz de compatibilidad SO / Python

Hay **tres** entornos distintos; cada uno con su Python, y conviene no confundirlos:

| Entorno | SO | Python | Qué corre | Estado |
|---|---|---|---|---|
| **Desarrollo / CI** | macOS / Linux | **3.10–3.13** | suite de tests, simulación, entrenamiento | al día (CI prueba 3.10/3.11/3.12/3.13) |
| **Contenedor del robot** | Ubuntu 22.04 (`dt-ros-commons:ente`) | **3.10** | el stack ROS (percepción, planificación, control) | al día |
| **Host Jetson Nano (bare-metal)** | Ubuntu 18.04 (JetPack 4.6) | **3.6** | solo herramientas de diagnóstico (`tools/detect_sensors.py`, `tools/calibrate.py`, scripts de shell) | **EOL** (límite del HW) |

Notas:
- `requires-python = ">=3.10"` aplica al paquete/tests, no a las herramientas
  bare-metal del host (escritas compatibles con 3.6, que es lo que trae JetPack 4.6).
- El **contenedor** ya provee `numpy`/`pyyaml`/`smbus`; `requirements-robot.txt`
  pone topes (`numpy<2`) para que un build futuro no rompa la ABI de ROS Noetic.
- La **Jetson Nano** está descontinuada (JetPack 4.6 es el último; Ubuntu 18.04 y
  Python 3.6 son EOL). El camino "al día" del hardware es **Jetson Orin** o
  **Raspberry Pi 4/5** (SO moderno) — ver `docs/hardware-alternativo.md`; el stack ya
  abstrae la placa (`platform_support/board.py`).

## Nota sobre `dts`
`tools/install_dts.sh` y `tools/flash_sd.sh` (wrapper de `dts`) siguen disponibles
como atajo, pero **no son necesarios**: build/deploy/flash/run tienen alternativa
propia (secciones 3–5). Así controlamos todo el ciclo nosotros.
