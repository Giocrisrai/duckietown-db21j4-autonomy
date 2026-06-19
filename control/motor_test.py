"""Prueba SEGURA del control de motores del Duckiebot (logica pura, testeable).

Genera una secuencia acotada (adelante/atras/giros/parar) para validar ambos
motores sin riesgo. El runner robot-side la envia via el driver real (PCA9685 I2C
o wheels_driver de dt-duckiebot-interface). Siempre termina en PARAR (seguridad).
"""

VMAX = 0.5


def clamp_velocity(v, vmax=VMAX):
    """Acota la velocidad de rueda a [-vmax, vmax] por seguridad."""
    return max(-vmax, min(vmax, v))


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
