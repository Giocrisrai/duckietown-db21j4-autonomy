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
| Calibración cinemática (gain/trim/baseline/radius) | **Implementada** con carga/guardado/validación YAML formato Duckietown (`calibration.py`, TDD) + **CLI unificada** (`tools/calibrate.py generate/validate/show`) + base de fábrica generada (`config/calibrations/`) | ✅⭐ |
| Calibración de cámara (intrínsecos + undistort fisheye) | **Implementada** (`camera_calibration.py`: validar/cargar/**guardar** + undistort cv2, TDD) + base por modelo (`tools/calibrate.py --camera`) | ✅⭐ |

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
| CI matriz (Ubuntu+macOS, py3.10-3.13) + Docker | ⭐ plus |
| Transfer learning validado (+0.273 mAP) | ⭐ plus |
| Diagnóstico de fallas HW + red | ⭐ plus |
| Monitoreo de salud EN VIVO por red (HTTP JSON, debounce, OK/DEGRADED/FAULT) | ⭐ plus |
| Validación E2E de hardware (preflight `validate_hardware.sh`) | ⭐ plus |
| Soporte de cualquier modelo de cámara (autodetección, no limitante) | ⭐ plus |

## Arquitectura HUT del DB21-J4 (verificado leyendo el driver oficial)
- **Motores**: PCA9685 PWM por **I2C** (0x60; el 0x40 es el de los LEDs) + **GPIO del Jetson** para dirección, vía el
  **header de 40 pines** (driver oficial `hat_driver/motor.py`: `Adafruit_PWM_Servo_Driver`).
- **IMU (0x68)/ToF (0x29)/OLED (0x3c)**: I2C por el **header de 40 pines** (`tools/detect_sensors.py`).
- **MCU/Duckiebattery USB** (Atmel `04d8:ecb7`, `/dev/ttyACM0`): power management/periféricos USB.
  Detección: `tools/check_hut.sh` + `diagnostics/hut.py`.
- **Cámara**: CSI directa al Jetson (nvargus, autodetección IMX219/OV5647/Pi-M12) o USB fallback.
- **DIAGNÓSTICO CLAVE**: si el **USB del MCU aparece pero el I2C está vacío**, el **header de
  40 pines no está mateado** -> el Jetson no está bien clavado en la HUT (reasentar a fondo).
- **Bus 1 COLGADO** (i2cdetect se cuelga) que **sobrevive al reset del controlador**
  (`unbind/bind` de `tegra-i2c`) **y** a alimentar con batería+fuente -> fallo **físico**:
  pin doblado puenteando SDA/SCL, chip I2C del HUT en corto, o HUT defectuosa.
  Cámara con `-121` en dmesg = **flex CSI mal contactado** (no es overlay). Procedimiento
  completo de aislamiento en [diagnostico-hardware-campo.md](diagnostico-hardware-campo.md).

## Gaps conocidos (pendientes honestos)
> Análisis completo y priorizado en [revision-profunda.md](revision-profunda.md).

| Funcionalidad oficial | Estado | Plan |
|---|---|---|
| **Línea roja de stop** (`stop_line_filter`) | ✅ módulo | `object_detection.stop_line` (HSV doble + ROI + distancia), 7 tests. Pendiente: cablear al navegador (nodo) |
| **Ground projection + filtro d/φ** (`lane_filter`) | 🔴 parcial | Proyección px→suelo (req. calib. extrínseca) + filtro bayesiano d/φ, o reusar `lane_filter` de dt-core |
| **Anti-Instagram** (color/iluminación) | ✅ módulo | `object_detection.anti_instagram` (transformación lineal por percentiles), 6 tests. Pendiente: cablear antes del `line_detector` (nodo) |
| **Calibración extrínseca de cámara** (homografía) | 🔴 falta | Añadir `camera_extrinsic/` + procedimiento `dts calibrate_extrinsics` (requisito de demos) |
| **FSM explícita** (`fsm`) | ✅ módulo | `duckietown_planning.fsm.DrivingFSM` (5 estados, seguridad prioritaria), 8 tests |
| **Fusión obstáculo visual+ToF** (`obstacle_detection`) | ✅ módulo | `duckietown_planning.obstacle_fusion` (visión+ToF → GO/SLOW/STOP graduado), 12 tests |
| Coordinación interseccion + LED (`explicit_coordinator`) | 🟡 falta | Solo si la prueba es multi-robot (paquetes en daffy, no en ente) |
| Teleop / joystick | ⚪ pendiente | Reusar `joy_mapper`/virtual joystick oficial vía launch |
| Captura fina de calibración (tablero) | ⚪ parcial | Base de fábrica + I/O está (`tools/calibrate.py`); captura con tablero 3.1 cm en el robot |
| Despliegue real en robot | ⚪ pendiente | Hardware: flashear, build TensorRT, conectar `~turn_cmd`→dt-car-interface |
| Segmentación neuronal de carril | ⚪ opcional | Por diseño CV clásica (la Nana solo aguanta una red, YOLO26) |
| gym-duckietown 3D oficial | ⚪ opcional | Frágil en M1/Py3.13; se usó entorno RL propio |
| Formato submission AIDO | ⚪ opcional | Solo si se compite en AIDO oficial (empaquetado Docker) |

## Verificación
- `tools/validate_stack.sh` → tests + launches + YAML + scripts + compose.
- `make sim` → simulación de planificación. Contenedor dev → suite en cualquier host.
- CI verde (Ubuntu+macOS+Docker).
