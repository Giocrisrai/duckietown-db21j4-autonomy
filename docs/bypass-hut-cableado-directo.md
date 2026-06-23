# Bypass de la HUT — cableado directo al Jetson (seguir desarrollando sin HUT)

Procedimiento para **seguir trabajando** cuando la sección I2C de la HUT está
dañada (línea SDA/SCL tomada en bajo — ver
[diagnostico-hardware-campo.md](diagnostico-hardware-campo.md) y el clasificador
`diagnostics/i2c_bus.py`). Codificado y testeado en `diagnostics/wiring.py`.

> Caso real `prueba`: la HUT quedó con el bus I2C muerto (culpable **soldado**:
> IMU o un PCA9685 — los enchufables ToF/OLED se descartaron uno por uno). La HUT
> debe reemplazarse; entretanto, este bypass permite validar el stack.

## Qué se recupera con bypass

| Subsistema | ¿Bypass directo? | Cómo |
|---|---|---|
| **Cámara** CSI (IMX219) | ✅ ya independiente de la HUT | funciona sin tocar nada |
| **Sensores I2C** (IMU 0x68, ToF 0x29, OLED 0x3c) | ✅ sí | breakouts al header (abajo) |
| **Motores** | ⚠️ con driver externo | PCA9685 externo @ 0x60 + puente H |
| LEDs RGB | ⚠️ con driver externo | igual que motores (PCA9685 LEDs) |

## 1. Sensores I2C directo al header de 40 pines

El bus **I2C-1** del Jetson (pines del header) ya trae las pull-ups del dev kit,
así que los breakouts funcionan sin la HUT. Todos comparten los **mismos 2 hilos**:

| Señal | Pin (header 40) |
|---|---|
| 3.3 V | **pin 1** |
| **SDA** (I2C-1) | **pin 3** |
| **SCL** (I2C-1) | **pin 5** |
| GND | **pin 6** |

- IMU **MPU-6050** (0x68), ToF **VL53L0X** (0x29), OLED **SSD1306** (0x3c) se
  encadenan en esos 4 pines (3.3 V y GND en común; SDA/SCL compartidos).
- Son de 3.3 V → compatibles directo con el header (no usar 5 V en SDA/SCL).
- **El stack los reconoce sin cambios**: escanea por dirección
  (`diagnostics/i2c_sensors.py`, `tools/detect_sensors.py --bus 1`).

Validar:
```bash
# en el robot (host Jetson)
i2cdetect -y -r 1            # deben aparecer 0x68 / 0x29 / 0x3c segun lo conectado
sudo python3 tools/detect_sensors.py 1    # bus posicional (por defecto 1)
```

## Arquitectura: perfil de hardware (bus + direcciones configurables)

El bus I2C y las direcciones **ya no están clavados** en el código: viven en un solo
lugar, `platform_support/hw_profile.py`, y se eligen con `DT_HW_PROFILE`. Así la
**misma lógica** de motores/sensores sirve sobre la HUT o sobre un driver externo en
otro puerto/dirección — sin reescribir nada.

| Perfil | Bus motores | Dir. motores | Cuándo |
|---|---|---|---|
| `hut` (def) | 1 (pines 3/5) | `0x60` | HUT sana |
| `external-bus1` | 1 (pines 3/5) | **`0x61`** | el `0x60` de la HUT murió pero el **bus 1 vive** (IMU responde): cuelgas un PCA9685 externo en `0x61` y reutilizas la IMU `0x68` |
| `external-gpio` | `i2c-gpio` nuevo | `0x60` | bus bit-bang en pines GPIO libres, **independiente de la HUT** |

```bash
make detect-motors PROFILE=external-bus1     # detecta el driver en el perfil elegido
DT_HW_PROFILE=external-bus1 sudo -E python3 tools/detect_motors.py
```

> Caso `prueba`: el bus 1 revivió y la **IMU (0x68) funciona**, pero **ambos PCA9685
> de la HUT (motores 0x60 / LEDs 0x40) hacen ACK pero NACKean toda lectura/escritura**
> (chips dañados). Camino real para mover motores: **`external-bus1`** (PCA9685
> externo en `0x61` sobre el bus 1 vivo) o **`external-gpio`** (bus nuevo, `0x60`).

## 2. Motores — driver externo (drop-in)

El Jetson **no** mueve un motor DC desde un GPIO (entrega µA; el motor pide
~0.5–2 A). Se necesita un **driver con puente H**. Camino drop-in con el stack:

- Usa un breakout **PCA9685 + TB6612FNG** (o un Motor HAT tipo Adafruit).
- **Pon el PCA9685 externo en la dirección `0x60`** → el código de motores del
  stack lo usa **sin cambiar una línea** (espera el PWM de motores en 0x60).
- Cableado:
  - PCA9685: VCC→3.3 V (pin 1 o 17), GND→GND, SDA→pin 3, SCL→pin 5.
  - PCA9685 PWM → entradas del TB6612; TB6612 → motores.
  - **Alimentación de motores desde la batería** (no desde el Jetson).
  - GND **común** entre Jetson, PCA9685, TB6612 y batería.

> `diagnostics/wiring.py::external_motor_driver_addr()` devuelve `0x60` y un test
> verifica que sigue siendo la dirección esperada del stack (drop-in garantizado).

## 3. Alimentación del Jetson sin la HUT

Antes de quitar la HUT, asegura cómo se alimenta el Jetson:
- Si la batería lo alimenta por **su propio USB/barrel** (independiente del header)
  → quitas la HUT y el Jetson sigue encendido.
- Si el Jetson se alimenta **a través de la HUT** → necesitas **5 V/4 A al barrel
  jack** (con el jumper **J48** puesto) o micro-USB.

## 4. Seguridad de motores

Igual que con la HUT: **no girar motores hasta confirmar ruedas en el aire**
(robot sobre soporte). Primero un pulso mínimo por rueda para verificar sentido,
luego el test completo (`tools/detect_motors.py`).

## Resumen rápido (generado por código)

`python3 -c "import sys; sys.path.insert(0,'diagnostics'); import wiring; print(wiring.wiring_summary())"`

```
Bypass de la HUT — cableado directo al header de 40 pines (bus I2C-1):
  SDA -> pin 3, SCL -> pin 5, 3V3 -> pin 1, GND -> pin 6.
  Sensores I2C directos (comparten el bus): MPU6050 (IMU) (0x68), VL53L0X (ToF) (0x29), SSD1306 (OLED) (0x3c).
  Motores: requieren DRIVER EXTERNO con puente H. Pon el PCA9685 externo en 0x60 y es drop-in con el stack.
  Camara: CSI, independiente de la HUT (ya funciona).
```
