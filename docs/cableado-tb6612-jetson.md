# Cableado: TB6612 manejado NATIVO por la Jetson (sin PCA9685)

La forma más "nuestra": manejar los motores con los periféricos **propios de la
Jetson** (GPIO + PWM por hardware) sobre un **TB6612**, sin ningún PCA9685 — el
análogo Jetson de lo que el colega hace con la GPIO de la Raspberry. Lo maneja
`control/tb6612.py` + `control/jetson_io.py` (con `Jetson.GPIO`).

## Lista de materiales (~US$2)
- 1× driver **TB6612FNG** (puente H doble).
- Cables dupont + la **batería del Duckiebot** (potencia de motores).
- (Los motores se recablean desde la HUT muerta hacia este TB6612.)

## Mapa de pines (header 40-pin, numeración BOARD) — `DEFAULT_PINS`
```
  Motor IZQUIERDO            Motor DERECHO
  ---------------            -------------
  PWM  -> pin 32 (PWM0)      PWM  -> pin 33 (PWM2)
  IN1  -> pin 29 (GPIO)      IN1  -> pin 35 (GPIO)
  IN2  -> pin 31 (GPIO)      IN2  -> pin 37 (GPIO)
```
> PWM por **hardware** en 32/33: habilitar UNA vez con
> `sudo /opt/nvidia/jetson-io/jetson-io.py` → enable `pwm0` y `pwm2` → reboot.
> (Si prefieres no tocar jetson-io, se puede PWM por software, pero con jitter.)

## Conexiones

### Jetson ↔ TB6612 (control)
```
  Jetson pin     TB6612      funcion
  ----------     ------      -------
  pin 32     ->  PWMA        velocidad IZQ
  pin 29     ->  AIN1   \    direccion IZQ
  pin 31     ->  AIN2   /
  pin 33     ->  PWMB        velocidad DER
  pin 35     ->  BIN1   \    direccion DER
  pin 37     ->  BIN2   /
  pin 1 (3.3V) -> VCC        logica del TB6612
  pin 17(3.3V) -> STBY       enable (si queda bajo, no mueve nada)
  pin 6 (GND)  -> GND        tierra comun
```

### TB6612 ↔ potencia y motores
```
  VM        -> + BATERIA (voltaje de motores, 6-12V)
  GND       -> GND comun (Jetson + batería -)
  AO1, AO2  -> motor IZQUIERDO
  BO1, BO2  -> motor DERECHO
```

⚠️ **GND común** entre Jetson, TB6612 y batería, obligatorio.
⚠️ Si la HUT sigue puesta (para usar la IMU 0x68), estos pines (29/31/32/33/35/37)
   no deben chocar con los que ella usa; si chocan, retira la HUT y usa solo la
   Jetson + este TB6612.

## Puesta en marcha (RUEDAS EN EL AIRE)
```bash
# en el robot (Jetson.GPIO instalado: pip3 install Jetson.GPIO)
sudo python3 - <<'PY'
import sys; sys.path.insert(0,'/home/prueba/duckietown_new_version/control')
from jetson_io import JetsonIO, DEFAULT_PINS
from tb6612 import TB6612Motor
io = JetsonIO()
L = TB6612Motor(io, **DEFAULT_PINS["left"])
R = TB6612Motor(io, **DEFAULT_PINS["right"])
import time
L.run(0.3); R.run(0.3); time.sleep(1)      # adelante suave
L.run(-0.3); R.run(-0.3); time.sleep(1)    # atras
L.stop(); R.stop(); io.cleanup()
print("OK")
PY
```
Si un motor gira al revés: intercambia AO1<->AO2 (o BO1<->BO2) o el signo del duty.

## Qué reusa del stack (sin tocar código)
- `control/tb6612.py` (lógica de puente H) + `control/jetson_io.py` (Jetson.GPIO).
- `control/kinematics.py` (v,ω→duty) → `tb6612` por rueda.
- `control/motor_safety.py` (watchdog), `lane_follow_loop.py` (lazo de marcha),
  `maneuver.py` (giros por demostración). Todo testeado (suite completa).
