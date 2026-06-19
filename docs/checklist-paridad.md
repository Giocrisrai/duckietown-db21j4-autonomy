# Checklist de paridad con Duckietown original + mejoras

Auditoría de funcionalidades del Duckietown DB21-J4 **oficial** vs **nuestra versión**.
Estados: ✅ paridad · ⭐ mejorado/plus · ♻️ reusa el driver oficial (no reescrito) · ⚠️ pendiente.

## Drivers de hardware
| Funcionalidad oficial | Nuestra versión | Estado |
|---|---|---|
| Cámara CSI (IMX219) | Reusa `camera_node` + **diagnóstico** (`check_camera.sh`, `camera_health`) + **soporte MULTI-CÁMARA** (IMX219/IMX219-B/IMX477/USB, `camera_support.py`, `probe_camera.sh`) | ♻️⭐ |
| Motores / `wheels_driver` | Reusa `dt-duckiebot-interface` | ♻️ |
| Encoders de rueda | Reusa drivers + **odometría/deadreckoning propia** + health | ♻️⭐ |
| IMU (MPU-6050) | Reusa driver + **chequeo de salud** | ♻️⭐ |
| ToF (VL53L0X) | Reusa driver + **freno de emergencia fail-safe** | ♻️⭐ |
| LEDs RGB / `led_emitter` | Reusa driver | ♻️ |
| OLED display / botón | Reusa driver | ♻️ |

## Autonomía
| Funcionalidad oficial | Nuestra versión | Estado |
|---|---|---|
| Seguir línea (line_detector + lane_controller PID) | Reusa visión clásica + **PID propio simulable/tuneable** | ♻️⭐ |
| Navegación de intersecciones (FSM, stop_line) | **Navegador reactivo propio** (detección de flanco) | ⭐ |
| Planificación de ruta global | **Planeador A*/BFS propio** (no existe en el básico oficial) | ⭐ plus |
| Detección de objetos (YOLOv5 oficial) | **YOLO26** + TensorRT + sim ONNX (mAP 0.937) | ⭐ |
| AprilTags / señales | **Tabla oficial + detección real + sim + enlace a planificación** | ⭐ |
| Odometría / deadreckoning | **Implementada** (`odometry.py`, TDD) | ✅⭐ |
| Calibración cinemática (gain/trim/baseline/radius) | **Implementada** con carga/guardado/validación YAML formato Duckietown (`calibration.py`, TDD) | ✅⭐ |
| Calibración de cámara (intrínsecos + undistort fisheye) | **Implementada** (`camera_calibration.py`: validar/cargar + undistort cv2, TDD) | ✅⭐ |

## Simulación y validación (antes de flashear)
| Capacidad | Nuestra versión | Estado |
|---|---|---|
| Simulador 3D (gym-duckietown) | **Entorno RL propio (Gymnasium)** + PPO + analítica | ⭐ (alternativa robusta) |
| Simulación de planificación | **Lazo cerrado** (`make sim`) | ⭐ |
| Simulación de percepción | **ONNX end-to-end** sin robot | ⭐ |
| Validación integral | **`validate_stack.sh`** (tests + configs + compose) | ⭐ plus |

## Plataforma / DevOps (plus sobre el oficial)
| Capacidad | Estado |
|---|---|
| Independencia de `dts` (build/deploy/flash propios) | ⭐ plus |
| Multiplataforma (cualquier Linux/macOS) + contenedor dev | ⭐ plus |
| Gestor `uv` (estado del arte) | ⭐ plus |
| CI matriz (Ubuntu+macOS, py3.11/3.12) + Docker | ⭐ plus |
| Transfer learning validado (+0.273 mAP) | ⭐ plus |
| Diagnóstico de fallas HW + red | ⭐ plus |
| Monitoreo de salud EN VIVO por red (HTTP JSON, debounce, OK/DEGRADED/FAULT) | ⭐ plus |
| Validación E2E de hardware (preflight `validate_hardware.sh`) | ⭐ plus |
| Soporte de cualquier modelo de cámara (autodetección, no limitante) | ⭐ plus |

## Gaps conocidos (pendientes honestos)
| Funcionalidad oficial | Estado | Plan |
|---|---|---|
| Teleop / joystick | ⚠️ pendiente | Reusar `joy_mapper`/virtual joystick oficial vía launch (no requiere código propio) |
| Procedimiento de captura de calibración de cámara (tablero) | ⚠️ parcial | El manejo (validar/cargar/undistort) está; la captura con tablero se hace en el robot |
| Despliegue real en robot | ⚠️ pendiente | Requiere hardware: flashear, build TensorRT, conectar `~turn_cmd`→dt-car-interface |
| Segmentación neuronal de carril | ⚠️ opcional | Por diseño usamos CV clásica (la Nana solo aguanta una red, YOLO26) |
| gym-duckietown 3D oficial | ⚠️ opcional | Frágil en M1/Py3.13; se usó entorno RL propio |

## Verificación
- `tools/validate_stack.sh` → tests + launches + YAML + scripts + compose.
- `make sim` → simulación de planificación. Contenedor dev → suite en cualquier host.
- CI verde (Ubuntu+macOS+Docker).
