# Cableado: PCA9685 externo + TB6612 (driver de motores propio)

Replica el enfoque del repo Pi del colega (driver raw, sin dts) pero sobre un
**PCA9685 externo que SÍ responde**, en el bus I2C-1 vivo del Jetson (la IMU 0x68
lo prueba). Lo maneja nuestro `control/pca9685.py` + el perfil `external-bus1`
(`platform_support/hw_profile.py`: motor en **0x61**, bus 1).

> Por qué externo: el PCA9685 0x60 de la HUT de `prueba` no responde a I2C
> (EREMOTEIO en toda escritura, incluido el código exacto del colega) — es
> hardware. Un PCA9685 externo en **0x61** esquiva el chip muerto sin tocar nada más.

## Lista de materiales (~US$5)
- 1× breakout **PCA9685** (16 canales PWM, I2C) — el típico de Adafruit/genérico.
- 1× driver **TB6612FNG** (puente H doble) — o un módulo con TB6612.
- Cables dupont, y la **batería del Duckiebot** para alimentar los motores.

## Direccion I2C del PCA9685 → 0x61
El breakout trae jumpers A0–A5 (dirección base 0x40). Para **0x61**:
- Puentea **A5** (suma 0x20) **y A0** (suma 0x01): 0x40 + 0x20 + 0x01 = **0x61**.
  (Equivale a la dirección de "motores" del HATv3 0x60, +1, para no chocar con el
  chip muerto que aún hace ACK en 0x60.)

## Conexiones

### 1) PCA9685 ↔ Jetson (header de 40 pines, bus I2C-1)
```
  PCA9685            Jetson 40-pin
  --------           -------------
  VCC (logica)  -->  pin 1  (3.3V)
  GND           -->  pin 6  (GND)
  SDA           -->  pin 3  (I2C1 SDA)
  SCL           -->  pin 5  (I2C1 SCL)
  V+ (potencia) -->  (no requerido para TB6612; dejar sin conectar o a 5V)
```

### 2) PCA9685 (salidas PWM) ↔ TB6612 (entradas de control)
Usamos 6 canales del PCA9685. Mapa (lo fija nuestro driver):
```
  PCA9685 canal      TB6612 pin        funcion
  -------------      ----------        -------
  ch0  --> PWMA      velocidad motor IZQUIERDO
  ch1  --> AIN1  \   direccion motor IZQUIERDO
  ch2  --> AIN2  /
  ch3  --> PWMB      velocidad motor DERECHO
  ch4  --> BIN1  \   direccion motor DERECHO
  ch5  --> BIN2  /
```
> Ventaja vs la HUT: aqui **ambos** motores van 100% por el PCA9685 (PWM+dir),
> sin el truco de GPIO/pinctrl para la dirección del motor derecho del HATv3.

### 3) TB6612 ↔ alimentación y motores
```
  TB6612 pin     a...
  ----------     ---
  VM        -->  + de la BATERIA (voltaje de motores, p.ej. 6-12V)
  VCC       -->  3.3V (logica)  [Jetson pin 1 o 17]
  GND       -->  GND comun (Jetson GND + batería -)
  STBY      -->  3.3V  (enable; si queda en bajo, el driver no mueve nada)
  AO1, AO2  -->  motor IZQUIERDO
  BO1, BO2  -->  motor DERECHO
```

⚠️ **GND comun obligatorio** entre Jetson, PCA9685, TB6612 y batería.
⚠️ La **velocidad** (PWM) por I2C; el TB6612 entrega la corriente. El Jetson nunca
   maneja el motor directo.

## Puesta en marcha (con ruedas EN EL AIRE)
```bash
# 1) confirmar que el PCA9685 externo responde en 0x61
make detect-motors PROFILE=external-bus1
#    -> "Driver de motores en bus 1 0x61 (perfil 'external-bus1')"

# 2) validar todo sin mover (preflight)
make validate-robot

# 3) RUEDAS EN EL AIRE -> prueba de giro (PWM <=40%, libera al final)
DT_HW_PROFILE=external-bus1 make validate-robot-motors
```
Si el motor gira al revés: intercambia AO1<->AO2 (o BO1<->BO2), o ajusta el signo
en el mapa de canales del driver.

## Qué reusa del stack (sin tocar código)
- `control/pca9685.py` — driver (init + set_duty), bus inyectable.
- `platform_support/hw_profile.py` — perfil `external-bus1` (bus 1, 0x61).
- `control/kinematics.py` (v,ω→duty) + `motor_safety.py` (watchdog) +
  `lane_follow_loop.py` (lazo de marcha) + `maneuver.py` (giros por demostración).
- Todo testeado (382 tests). Solo conectas el hardware y corre.
