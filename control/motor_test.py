"""Prueba SEGURA del control de motores del Duckiebot (logica pura, testeable).

Genera una secuencia acotada (adelante/atras/giros/parar) para validar ambos
motores sin riesgo. El runner robot-side la envia via el driver real (PCA9685 I2C
o wheels_driver de dt-duckiebot-interface). Siempre termina en PARAR (seguridad).
"""

VMAX = 0.5
SPEED_MAX = 255          # rango de speed del driver oficial Motor.set (0..255)
SPEED_CAP = 0.40         # tope de seguridad: como maximo 40% del PWM en las pruebas


def clamp_velocity(v, vmax=VMAX):
    """Acota la velocidad de rueda a [-vmax, vmax] por seguridad."""
    return max(-vmax, min(vmax, v))


def velocity_to_command(v, vmax=VMAX, speed_cap=SPEED_CAP):
    """Traduce velocidad de rueda [-vmax, vmax] al API del driver oficial.

    Devuelve (direccion, speed) donde direccion in {"FORWARD","BACKWARD",
    "RELEASE"} y speed es entero 0..255 (Motor.set del hat_driver oficial).
    `speed_cap` limita el PWM por seguridad en las pruebas (40% por defecto).
    """
    v = clamp_velocity(v, vmax)
    if v == 0:
        return "RELEASE", 0
    direction = "FORWARD" if v > 0 else "BACKWARD"
    speed = int(round(abs(v) / vmax * SPEED_MAX * speed_cap))
    speed = max(0, min(SPEED_MAX, speed))
    return direction, speed


def pulse_sequence(v=0.3, dt=0.5):
    """Secuencia [(v_left, v_right, duracion_s)] para probar ambos motores.

    Adelante, parar, atras, parar, giro izquierda, giro derecha, PARAR final.
    Las velocidades se acotan a VMAX.
    """
    v = clamp_velocity(v)
    return [
        (v, v, dt),        # adelante
        (0.0, 0.0, dt),    # parar
        (-v, -v, dt),      # atras
        (0.0, 0.0, dt),    # parar
        (-v, v, dt),       # giro izquierda
        (v, -v, dt),       # giro derecha
        (0.0, 0.0, dt),    # PARAR (seguridad)
    ]
