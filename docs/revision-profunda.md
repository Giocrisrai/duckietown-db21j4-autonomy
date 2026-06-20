# Revisión profunda — análisis de brechas vs Duckietown oficial

Revisión "desde cero" del stack, contrastado con el software oficial de Duckietown
(`dt-core`, `dt-duckiebot-interface`, `dt-car-interface`), su documentación
(`docs.duckietown.com`), la especificación de pista/señales, y las reglas AIDO.
Objetivo: ver qué nos faltó, qué conviene incorporar y qué NO copiar.

> Fecha: 2026-06-19. Distro de referencia: **`ente`** (sucesora de `daffy`).

## Resumen ejecutivo (veredicto honesto)

- **Capa algorítmica/CPU — sólida y bien testeada (173 tests):** percepción
  (letterbox/postproceso, multi-cámara, calibración, AprilTags), planificación
  (intersección reactiva, grafo+A*/BFS, localización, odometría, safety ToF),
  control (PID, teleop, secuencia de motores), simulación, diagnósticos, RL.
- **Capa de integración real con el robot — implementada pero NO verificada en
  hardware:** los 8 nodos ROS (percepción, apriltags, freno, navegador, planeador)
  tienen 0 tests y nunca corrieron en un robot sano; el engine **TensorRT** nunca
  se validó con un modelo real; el lazo de marcha se apoya en el `lane_controller`
  **externo** de `dt-core`. (El robot `prueba` tiene el bus I2C-1 caído y la cámara
  con falla de contacto — ver [informe-hardware-prueba.md](informe-hardware-prueba.md).)
- **Brechas funcionales reales frente al oficial** (detalle abajo): falta detección
  de **línea roja de stop**, **ground projection + filtro d/φ**, **anti-Instagram**
  (robustez a iluminación), **FSM explícita**, **fusión visual+ToF** de obstáculos,
  **calibración extrínseca** de cámara y la **captura real** de calibración.

## 1. Lo que ya cubrimos (y dónde superamos al oficial)

| Capacidad oficial | Nuestra versión | Estado |
|---|---|---|
| `line_detector` (segmentos de carril) | reutilizado de dt-core | ✅ paridad |
| `lane_control` (PID v,ω) | `control/pid.py` propio simulable | ✅⭐ |
| `vehicle_detection` (círculos traseros) | **YOLO26→TensorRT** (patos/coches/robots) | ⭐ supera |
| `navigation` (grafo de intersección) | planeador **A*/BFS** propio + reactivo | ⭐ supera |
| `apriltag` (detección+semántica) | tabla oficial + detección + sim | ✅⭐ |
| `kinematics` (dif. drive) | `calibration.py` + `tools/calibrate.py` | ✅⭐ |
| odometría / deadreckoning | `odometry.py` (TDD) | ✅⭐ |
| ToF / freno | `safety.py` fail-safe (NaN→frena) | ✅⭐ |
| drivers (ruedas/cámara/HAT/IMU/ToF) | abstracción de placa + multi-cámara | ✅⭐ |
| `gym-duckietown` (sim RL) | entorno **Gymnasium+PPO** propio | ⭐ alternativa |

## 2. Brechas priorizadas (qué nos faltó)

### 🔴 Alta prioridad (afecta completar una vuelta con reglas)

| Brecha | Por qué importa | Esfuerzo | Acción |
|---|---|---|---|
| ✅ **Línea roja de stop** (`stop_line_filter`) | Las intersecciones se delimitan por línea **roja en el piso**, no por obstáculo. Sin esto el robot no sabe *dónde* parar en una intersección vacía. Hoy solo frenamos por ToF. | Medio | **HECHO:** `object_detection.stop_line` (máscara HSV doble + ROI frontal + fracción + proxy de distancia), 7 tests. Pendiente: cablear al navegador de intersección (nodo). |
| **Ground projection + filtro d/φ** (`ground_projection`+`lane_filter`) | Convierte píxeles→metros con la homografía y estima distancia al centro `d` y ángulo `phi` con un filtro bayesiano. Es el núcleo robusto del seguir-línea oficial. | Medio-Alto | Implementar proyección a suelo (requiere calibración **extrínseca**) + filtro de histograma d/φ, o reutilizar `lane_filter` de dt-core. |
| ✅ **Anti-Instagram** (corrección color/iluminación) | La detección clásica de línea (que reutilizamos) usa umbrales HSV que se rompen con cambios de luz. **Barato y de altísimo impacto en robustez.** | Bajo | **HECHO:** `object_detection.anti_instagram` (transformación lineal por canal por percentiles), 6 tests. Pendiente: cablear antes del `line_detector` (nodo). |
| **Calibración extrínseca + captura real** | No tenemos `camera_extrinsic/` ni el procedimiento con tablero (3.1 cm). Sin extrínsecos la proyección a pista degrada; además es **requisito** de las demos. | Medio | Doc `calibracion-robot.md` + carpeta `camera_extrinsic/` + validador de homografía. La captura fina (`dts calibrate_intrinsics/extrinsics`) corre en el robot. |

### 🟡 Media prioridad (escenarios LFI/LFV con intersecciones u otros robots)

| Brecha | Por qué importa | Acción |
|---|---|---|
| **FSM explícita** (`fsm`) | Hoy es reactivo. Una FSM central (LANE_FOLLOWING ↔ INTERSECTION ↔ COORDINATION ↔ STOP) hace el comportamiento predecible y depurable. | Modelar la FSM como módulo testeable que orqueste qué política está activa. |
| **Fusión obstáculo visual+ToF** (`obstacle_detection`) | El oficial hace OR de detección **visual** (vehículo/pato) con ToF → "virtual stop line". Nosotros tenemos YOLO26 y ToF por separado: falta el nodo que combine "veo un pato a X" → frenado anticipado suave. | Nodo de fusión YOLO26+ToF → señal de frenado graduada. |
| **Coordinación en intersección** (`explicit_coordinator`+`led_emitter/detection`) | Para intersecciones tipo semáforo y paso negociado **entre Duckiebots**. Obligatorio si la prueba es multi-robot; estos paquetes están en `daffy`, no en `ente`. | Solo si la competencia es multi-robot: portar desde daffy o implementar protocolo LED. |
| **Señalización por LED** (`led_emitter`) | Señalizar giro/parada; puntúa en pruebas de coordinación. | Patrones LED semánticos sobre el driver de LED existente. |
| **Verificar nuestro `apriltagsDB.yaml` vs el oficial** | Confirmar familia **tag36h11**, IDs (1=stop, 8=4-way, 9/10/11=T-intersect, 12=pedestrian, 13=semáforo) y medidas físicas (6.5 cm lado, 0.8 cm borde, 13 cm altura). | Diff contra `duckietown/signs-and-tags`; documentar medidas. |

### 🟢 Baja prioridad / infraestructura

- **Formato de submission AIDO** (`challenge-aido_LF-*` templates): solo si se compite en AIDO oficial; es empaquetado Docker, no capacidad de conducción.
- **OLED/botón** (`display_driver`/`button_driver`): calidad de vida operacional.
- **Alinear nombres `dts`** (`init_sd_card`, dashboard, rutas `/data/config/calibrations/`) en `docs/flasheo.md`.

## 3. Estándares oficiales que debemos respetar (referencia condensada)

### Pista y señalización (Appearance Spec)
- Carril: amarilla **discontinua** al centro (~2 in pieza / 1 in gap), blanca **sólida**
  al borde derecho, **roja** de stop **solo en tiles de intersección** (ancho ~4.8 cm,
  perpendicular cruzando el carril).
- Intersecciones: desde cada línea de stop se ven **2 señales** — tipo (semáforo o
  stop) y topología (3-way con orientación, o 4-way).
- *(Verificar en el manual las dimensiones exactas de tile/carril antes de construir pista.)*

### AprilTags
- Familia **tag36h11**; señales **6.5 cm** de lado, borde blanco **0.8 cm**, centro a
  **13 cm** del piso, perpendiculares al suelo. Mapeo ID→significado en el
  `apriltagsDB.yaml` oficial (`tag_type`: TrafficSign / StreetName / Localization / Vehicle).

### Calibración (procedimiento real en el robot)
- **Ruedas (cinemática):** línea de cinta, comandar ~2 m, medir deriva, ajustar
  **trim** (deriva) y **gain** (velocidad) hasta deriva < 10 cm;
  `rosservice call …/kinematics_node/save_calibration`. → `kinematics/[host].yaml`.
- **Cámara intrínseca:** `dts duckiebot calibrate_intrinsics [host]`, tablero **3.1 cm**,
  barras X/Y/Size/Skew verdes → COMMIT. → `camera_intrinsic/[host].yaml`.
- **Cámara extrínseca (ground projection):** `dts duckiebot calibrate_extrinsics [host]`,
  robot sobre el tablero con el **eje de ruedas alineado al eje Y** → homografía.
  → `camera_extrinsic/[host].yaml`. **Requisito previo de las demos.**

### Reglas y métricas AIDO
- Retos: **LF** (solo carril), **LFV** (+vehículos/obstáculos), **LFI** (+intersecciones),
  **LFVI** (todo).
- 4 métricas oficiales: `survival_time_median` (↑ mejor), `driven_lanedir_consec_median`
  (↑ mejor, distancia **en el carril correcto**), `deviation-center-line_median` (↓ mejor),
  `in-drivable-lane_median` (↓ mejor). **Conectar la recompensa del RL** a
  `driven_lanedir_consec` y penalizar `in-drivable-lane`/colisión.

## 4. Deuda de integración real (lo más importante a cerrar)

1. **Nodos ROS sin verificar (0 tests, nunca en robot sano):** `object_detection_node`,
   `apriltag_node`, `emergency_brake_node`, `intersection_navigator_node`,
   `route_planner_node`. Es la capa que de verdad conecta con el robot.
2. **Lazo navegador→giro real NO cerrado:** falta conectar `~turn_cmd` al control de
   intersecciones de `dt-core`/`dt-car-interface`.
3. **Path TensorRT de competencia sin probar:** `trt_inference.py` + `build_trt_engine.sh`
   solo corren en Jetson; toda la validación de inferencia fue ONNX/CPU con modelo dummy.
4. **El control de marcha depende del `lane_controller` externo de dt-core** (no incluido aquí).
5. **Hardware del robot `prueba` roto** (bus I2C-1, cámara CSI) → nada de actuación real validada.

## 5. Roadmap recomendado (orden sugerido)

1. **Robustez de visión, barato primero:** anti-Instagram → línea roja de stop →
   ground projection + filtro d/φ. (Las tres suben mucho la fiabilidad de la vuelta.)
2. **Calibración real:** documentar y ejecutar intrínseca + **extrínseca** con tablero;
   añadir `camera_extrinsic/` y su validador.
3. **FSM explícita** que orqueste lane-following / intersección / stop / (coordinación).
4. **Fusión YOLO26+ToF** para frenado anticipado ante patos/vehículos.
5. **Cerrar el lazo on-robot** (nodos ROS ↔ dt-core) y **probar el engine TensorRT** real,
   en cuanto haya una HUT sana — usar `tools/validate_robot.sh --motors`.
6. **Conectar el RL a las métricas AIDO** y, si se compite en AIDO, el formato de submission.
7. **Docs nuevos:** `calibracion-robot.md`, `aido-reglas-metricas.md`, `especificacion-pista.md`.

## Qué NO copiar del oficial
- Teleop por joystick (`joy_mapper`, `led_joy_mapper`, `car_cmd_switch` manual).
- `vehicle_detection`/`led_detection` clásicos (patrón de círculos / HSV) — **ya superados**
  por YOLO26.
- `gym-duckietown` como simulador de entrenamiento — ya tenemos RL propio (Gymnasium+PPO).
- Paquetes `daffy`-only ya retirados en `ente` (`complete_image_pipeline`, `deadreckoning`
  suelto, etc.); usar el equivalente `ente` si se necesita la función.
- Configs `duckiedrone` (no aplican al DB21-J4).
