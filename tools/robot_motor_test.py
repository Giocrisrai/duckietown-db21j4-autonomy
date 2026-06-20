#!/usr/bin/env python3
"""Prueba REAL de motores del DB21-J4 con el driver OFICIAL (sin ROS).

Reutiliza `hat_driver` de dt-duckiebot-interface (HATv3: PCA9685 en 0x60, motor 1
direccion por PWM, motor 2 direccion por GPIO board-pins 33/31) y corre la
secuencia SEGURA de `control/motor_test.py`. Pensado para ejecutarse DENTRO del
contenedor dt-duckiebot-interface, donde estan `Adafruit_PWM_Servo_Driver`,
`Jetson.GPIO` y `dt_device_utils`:

    sudo docker run --rm --privileged \
        -v /home/prueba/duckietown_new_version:/work \
        --entrypoint bash duckietown/dt-duckiebot-interface:daffy-arm64v8 -lc \
        'export PYTHONPATH=/code/catkin_ws/src/dt-duckiebot-interface/packages/hat_driver/include:/work/control && \
         python3 /work/tools/robot_motor_test.py'

SEGURIDAD: ruedas en el aire / robot asegurado. PWM tope 40% (SPEED_CAP). Siempre
termina liberando ambos motores (RELEASE).
"""
import sys
import time

# logica pura y testeada (secuencia + traduccion velocidad->comando)
from motor_test import pulse_sequence, velocity_to_command


def _build_hat():
    """Instancia el HAT oficial. Devuelve (hat, MotorDirection) o lanza."""
    from hat_driver.hat import HATv3
    from hat_driver.motor import MotorDirection
    hat = HATv3()                       # PCA9685 @0x60, 1600Hz (default oficial)
    return hat, MotorDirection


def run_sequence(v=0.3, dt=0.5):
    """Corre la secuencia segura por el driver oficial. Devuelve 0 si OK."""
    hat, MotorDirection = _build_hat()
    dir_map = {
        "FORWARD": MotorDirection.FORWARD,
        "BACKWARD": MotorDirection.BACKWARD,
        "RELEASE": MotorDirection.RELEASE,
    }
    left = hat.get_motor(1, "left")     # motor 1: direccion PWM
    right = hat.get_motor(2, "right")   # motor 2: direccion GPIO (DB21-J4)
    print(f"  {left}")
    print(f"  {right}")
    try:
        for vl, vr, d in pulse_sequence(v=v, dt=dt):
            dl, sl = velocity_to_command(vl)
            dr, sr = velocity_to_command(vr)
            print(f"    L:{dl}@{sl:<3}  R:{dr}@{sr:<3}  ({d}s)")
            left.set(dir_map[dl], sl)
            right.set(dir_map[dr], sr)
            time.sleep(d)
    finally:
        # seguridad: liberar SIEMPRE ambos motores
        left.set(MotorDirection.RELEASE, 0)
        right.set(MotorDirection.RELEASE, 0)
        print("  Motores liberados (RELEASE).")
    return 0


def main():
    try:
        return run_sequence()
    except ImportError as e:
        print(f"  Driver oficial no disponible aqui ({e}).")
        print("  Ejecuta este script DENTRO del contenedor dt-duckiebot-interface.")
        return 2
    except Exception as e:
        print(f"  FALLO al comandar motores: {e}")
        print("  (Si el bus I2C 1 esta colgado o el PCA9685 0x60 no responde, "
              "revisa el acople de 40 pines / la HUT.)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
