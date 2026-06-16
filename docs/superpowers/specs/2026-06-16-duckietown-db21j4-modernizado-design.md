# Diseño: Stack modernizado para Duckiebot DB21-J4 (`duckietown_new_version`)

**Fecha:** 2026-06-16
**Autor:** giocrisrai
**Estado:** Aprobado para planificación

---

## 1. Objetivo

Construir un stack de autonomía **moderno, robusto y extensible** para un Duckiebot **DB21-J4** (NVIDIA Jetson Nano 4GB, board P3450), reutilizando lo que ya funciona en Duckietown y modernizando exactamente donde aporta valor: **detección de objetos con YOLO26**, **planificación de ruta en dos capas** (reactiva + global), y un **workflow de desarrollo/entrenamiento reproducible** (Google Colab + MacBook Pro M1). Meta inmediata: llegar competitivo a una competencia estilo Duckietown en **~1 semana** (seguir línea + intersecciones + planificación de ruta + detección de objetos).

## 2. Principio rector: extensibilidad incremental

El repo se diseña **modular** para poder **ir incorporando progresivamente más componentes de los repos oficiales de Duckietown** (`dt-core`, `dt-duckiebot-interface`, `dt-car-interface`, etc.) y mejorarlos con el tiempo. Cada mejora futura entra como un módulo/nodo nuevo o como configuración, sin reescribir lo existente. No bifurcamos Duckietown: lo **extendemos** vía Docker y paquetes ROS propios.

## 3. Restricciones duras (verificadas, no negociables por hardware)

| Restricción | Valor | Implicación |
|---|---|---|
| SO del robot | JetPack 4.6.6 / Ubuntu 18.04 (EOL) | No se puede actualizar Ubuntu en esta placa. Es un techo físico. |
| CUDA en robot | 10.2 | No hay PyTorch 2.x con GPU. |
| Python en robot | 3.6.9 | Ultralytics moderno (≥3.8) NO corre nativo en el robot. |
| PyTorch máx con GPU en robot | 1.10.0 | Solo para casos extremos; preferimos no usar PyTorch en el robot. |
| Compute capability | 5.3 (Maxwell) | Wheels modernos no la soportan; TensorRT sí. |
| Inferencia en robot | **Solo TensorRT** | El modelo YOLO26 se entrena fuera y se ejecuta como engine TensorRT. |
| Middleware | **ROS 1 Noetic**, Docker (jerarquía ente) | No ROS2 en producción (DTPS/ROS2 aún no listos en Duckietown). |
| Entrenamiento | **Siempre fuera del robot** | Colab (GPU CUDA) + M1 (PyTorch MPS) para iterar y validar. |

**Consecuencia de diseño:** una sola red neuronal (YOLO26) corre en GPU; el carril se detecta con **visión clásica en CPU** para liberar la GPU y maximizar FPS y robustez.

## 4. Decisiones tomadas

- **Distro base:** `ente` (actual, soportada, ago 2025; trae `dts code`, simulador Duckiematrix, Viewer). Respaldo conservador: `daffy`.
- **Estrategia:** construir **sobre** ente; modernizar percepción + planificación + workflow.
- **Percepción:** **híbrida** → YOLO26+TensorRT para objetos (GPU) + visión clásica de color para líneas de carril e intersecciones (CPU).
- **Planificación:** **dos capas** → reactiva (base, siempre funciona) + global cargable (mapa + localización simple).
- **Entrenamiento:** Colab (GPU) como principal + M1 con MPS como alternativa local de iteración/validación. Mismo código en ambos.
- **Dataset:** oficial de Duckietown (clases `duckie`, `cone`, `truck`, `bus`) como punto de partida; ampliable con datos propios más adelante.
- **Mapa:** desconocido hasta el día de la competencia → el reactivo es la base; el planeador global se carga con un mapa armado en el lugar.

## 5. Arquitectura

```
duckietown_new_version/
├── training/        # Pipeline YOLO26 (Colab + M1): dataset → train → val → export ONNX
├── perception/      # Paquete ROS: object_detection_node (TensorRT) + config carril clásico
├── planning/        # Paquete ROS: intersection_navigator (reactivo) + route_planner (global)
├── bringup/         # Launch files + docker-compose sobre drivers oficiales
├── tools/           # Verificación robot, dts, flasheo, build engine TensorRT, validación
└── docs/            # Setup, flasheo, entrenamiento, runbook de competencia
```

### 5.1 `training/` — Pipeline YOLO26
- **Qué hace:** descarga/prepara el dataset oficial (formato YOLO), entrena `YOLO26n` con `ultralytics` 8.4.x, valida (mAP), exporta a ONNX.
- **Cómo se usa:** un notebook/script que corre idéntico en Colab (CUDA) y en M1 (MPS, `device="mps"`). Salida versionada: `weights/best.pt` → `weights/model.onnx`.
- **Depende de:** ultralytics 8.4.x, dataset oficial. **Produce:** `model.onnx` (interfaz hacia `perception/`).

### 5.2 `perception/` — Detección de objetos + carril
- **`object_detection_node`** (ROS): carga el **engine TensorRT** (construido en la Nana desde `model.onnx`), suscribe a la cámara, publica detecciones (clase, bbox, confianza).
- **Carril:** reutiliza el `line_detector` clásico de `dt-core` (configurado, no reescrito). Corre en CPU.
- **Depende de:** `model.onnx` de `training/`, drivers de cámara de `dt-duckiebot-interface`. **Produce:** tópico de detecciones + segmentos de carril.

### 5.3 `planning/` — Planificación en dos capas
- **`intersection_navigator` (reactivo, base):** al detectar línea roja/intersección, elige giro (izq/recto/der) por política. **Siempre funciona, sin mapa.**
- **`route_planner` (global, opcional/stretch):** carga mapa de tiles (YAML estándar Duckietown), construye grafo, calcula secuencia de giros (A*/Dijkstra), con **localización simple** (contar intersecciones + encoders). Si no hay mapa → cae al reactivo automáticamente.
- **Depende de:** detecciones/carril de `perception/`, encoders de `dt-duckiebot-interface`. **Produce:** comandos de giro/velocidad hacia el control de `dt-car-interface`.

### 5.4 `bringup/`
- Launch files + `docker-compose` que orquestan los nodos propios junto a `dt-duckiebot-interface` y `dt-car-interface`. Punto único de arranque en el robot.

### 5.5 `tools/`
- Scripts de **verificación del estado del robot** (conectividad, cámara, motores), instalación de `dts`, wrapper de `dts init_sd_card --configuration DB21J`, **build del engine TensorRT** en la Nana, y checklists de validación.

## 6. Flujo de desarrollo y build

- **M1 (Mac):** desarrollo, entrenamiento/validación local (MPS), pruebas en simulador (gym-duckietown / Duckiematrix).
- **Colab:** entrenamiento completo de YOLO26 en GPU.
- **Build arm64** para la Nana con `dts devel build` / `docker buildx`.
- **Flasheo:** `dts init_sd_card` con imagen JetPack 4.6.6, configuración DB21J. Credenciales por defecto `duckie`/`quackquack`.

## 7. Pruebas / validación

- **Unit tests (pytest, en Mac):** lógica de grafo, A*/Dijkstra, política de intersecciones.
- **Validación YOLO26:** métricas mAP en dataset oficial **antes** de exportar a ONNX/TensorRT.
- **Simulador:** lane following + detección antes de tocar el robot.
- **Robot (smoke tests):** cámara, motores, FPS de detección (objetivo ~10–25 FPS), loop de seguir línea.
- **Runbook de competencia** con fallbacks en cascada.

## 8. Plan por fases (~1 semana)

| Fase | Entregable | Mínimo competitivo |
|---|---|---|
| **P0** | Verificar estado del robot, dts, reflasheo si hace falta | ✅ obligatorio |
| **P1** | Percepción YOLO26: entrenar → ONNX → engine TensorRT → nodo de detección | ✅ obligatorio |
| **P2** | Seguir línea sólido (visión clásica afinada) | ✅ obligatorio |
| **P3** | Intersecciones reactivas | ✅ obligatorio |
| **P4** | Planeador global (cargable en el lugar) | ⭐ stretch |
| **P5** | Integración + runbook de competencia | ✅ obligatorio |

## 9. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| Build de TensorRT en la Nana es delicado | Validarlo el **primer día**; guardar un engine "known-good" versionado. |
| Pista desconocida hasta el día | El **reactivo siempre funciona**; el global carga mapa en el venue. |
| Tiempo (1 semana) | Fases priorizadas: P0–P3 + P5 son el mínimo; P4 es stretch. |
| Estado del robot incierto | P0 es 100% diagnóstico antes de tocar nada. |
| FPS bajos con dos redes | Decisión de diseño: una sola red (YOLO26) en GPU + carril clásico en CPU. |

## 10. Fuera de alcance (YAGNI por ahora)

- Migración a ROS2 (Duckietown aún no lo soporta en producción).
- Actualizar Ubuntu/CUDA en el robot (imposible por hardware).
- Carril por red neuronal (riesgo de <5 FPS combinados en la Nana).
- Localización completa (SLAM); usamos localización simple por conteo de intersecciones/encoders.

---

## Apéndice A — Hechos verificados (investigación 2026-06-16)

**Jetson Nano P3450 (original 4GB):**
- Último JetPack: **4.6.6** / L4T 32.7.6 / Ubuntu 18.04 / CUDA 10.2. **EOL** (JetPack 4 declarado EOL 2024-11-22). PyTorch máx con GPU: **1.10.0** (Python 3.6). Compute capability 5.3. Placa discontinuada.

**Duckietown DB21-J4:**
- Compute: Jetson Nano **4GB** + HUT v3.15. Cámara CSI 160° FOV, 2× motores DC con encoders (135 ticks/rev), ToF **VL53L0X**, IMU **MPU-6050**, OLED I2C, 4× LED RGB, batería 10 Ah.
- Stack: **ROS 1 Noetic**, distro actual **`ente`** (2025-08-31), `dts` v6.x. Repos: `dt-base-environment` → `dt-commons` → `dt-ros-commons` → `dt-duckiebot-interface`/`dt-car-interface`/`dt-core`.
- Flasheo: `dts init_sd_card --configuration DB21J`; imagen JetPack 4.6.6.
- Detección oficial (curso): **YOLOv5n** → TensorRT; clases `duckie`, `cone`, `truck`, `bus`.
- Competencia AI-DO: pausada desde NeurIPS 2021 (la competencia objetivo es probablemente local/DUOC con reglas propias).

**YOLO:**
- **YOLO26** existe y es real (Ultralytics, **enero 2026**, NMS-free end-to-end). También YOLO12 (feb 2025). La numeración va 11 → 12 → **26** (por año).
- `ultralytics` 8.4.x requiere **Python ≥3.8** y PyTorch ≥1.8 → **no corre nativo en el robot** (Python 3.6). Camino correcto: entrenar fuera → exportar ONNX → **engine TensorRT** en la Nana. FPS realista: **~10–25** con modelos nano.

### Fuentes
- https://forums.developer.nvidia.com/t/announcing-end-of-life-for-nvidia-jetpack-4-with-the-release-of-jetpack-4-6-6/314300
- https://developer.nvidia.com/embedded/jetpack-archive
- https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- https://docs.duckietown.com/ente/duckietown-manual/10-setup/03-duckiebot/what-is-in-the-duckiebot-db21j4-box.html
- https://docs.duckietown.com/ente/duckietown-manual/10-setup/03-duckiebot/flashing-sd-card-duckiebot-initialization-complete.html
- https://github.com/duckietown/duckietown-lx/tree/mooc2022/object-detection
- https://www.ultralytics.com/news/ultralytics-redefines-state-of-the-art-vision-ai-with-yolo26
- https://docs.ultralytics.com/guides/nvidia-jetson/
- https://github.com/dusty-nv/jetson-inference
