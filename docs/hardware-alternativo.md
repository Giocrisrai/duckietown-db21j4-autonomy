# Hardware alternativo (Jetson Nano/Orin, Raspberry Pi) — stack agnostico

El stack no esta atado a la Jetson Nano: la **abstraccion de placa**
(`platform_support/board.py`) permite correr la misma autonomia en otras placas
potentes, cambiando solo el backend de inferencia y la camara.

## Placas soportadas
| Placa | Inferencia | Cámara | Arch | Imagen base |
|---|---|---|---|---|
| `jetson-nano` | TensorRT (GPU, CUDA 10.2) | CSI nvargus | arm64 | duckietown/dt-ros-commons |
| `jetson-orin` | TensorRT (GPU, mucho más potente) | CSI nvargus | arm64 | duckietown/dt-ros-commons |
| `raspberry-pi` | **ONNX Runtime (CPU)**; opcional Coral TPU | CSI libcamera | arm64 | ros:noetic-ros-base |
| `generic` (PC) | ONNX Runtime (CPU) | USB/UVC | amd64 | ros:noetic-ros-base |

Detección automática:
```python
from board import detect_board, inference_backend
b = detect_board(open("/proc/device-tree/model").read())   # en el robot
backend = inference_backend(b)     # 'tensorrt' o 'onnx'
```

## Mismo modelo, distinto backend (clave de la portabilidad)
Se entrena UN modelo YOLO26 (transfer learning) y se despliega segun la placa:
- **Jetson**: exportar ONNX → `build_trt_engine.sh` → `TRTDetector` (GPU).
- **Raspberry Pi / PC**: usar el **.onnx directamente** con `ONNXDetector` (CPU), sin CUDA.
```python
from object_detection.inference_backend import select_backend, make_detector
backend = select_backend(board)            # auto por placa
det = make_detector(modelo, backend=backend)   # TRTDetector u ONNXDetector
```

## Despliegue real (sin dts, multiplataforma)
1. **Validar todo** antes (sin hardware): `tools/validate_stack.sh` + `make sim` (todo verde).
2. **Construir** la imagen para la arch de la placa:
   `BOARD=raspberry-pi tools/build_image.sh` (cross-build arm64 con buildx).
   - Jetson usa la base de Duckietown; Pi/PC usan `ros:noetic-ros-base`
     (ajustar `--build-arg BASE_IMAGE` / Dockerfile segun la placa).
3. **Desplegar** y levantar contenedores: `tools/deploy_robot.sh tar <host> robot.tar`
   (o `registry`), que hace `docker compose up -d` en la placa.
4. **Preflight de hardware**: `tools/validate_hardware.sh <host>`.
5. **Monitoreo en vivo por red**: `python -m run_monitor` en la placa →
   `curl http://<host>:8080/`.
6. **Cámara**: autodetección (`auto_pipeline`); en Pi usa `libcamera`, en Jetson `nvargus`,
   con fallback USB. El modelo de cámara no limita.
7. **Calibrar** en pista (cinemática + cámara) y operar.

## Por qué Raspberry Pi es viable
- ONNX Runtime corre YOLO26n nano en CPU ARM (FPS menor que TensorRT pero usable a
  imgsz bajo); con un Coral USB TPU sube bastante.
- Sin las restricciones EOL de la Jetson Nano (Pi 4/5 tienen SO moderno y soporte).
- Misma autonomia, planificacion, AprilTags, PID y monitoreo: todo es agnostico.
