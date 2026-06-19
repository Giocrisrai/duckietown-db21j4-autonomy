"""Calibracion cinematica de ruedas del Duckiebot (trim/gain), como el oficial.

- gain: escala global de velocidad (ambas ruedas).
- trim: corrige asimetria izquierda/derecha (rueda que jala).
"""

def apply_trim_gain(v_left, v_right, trim=0.0, gain=1.0):
    """Aplica gain (escala) y trim (asimetria) a las velocidades de rueda.

    trim>0 acelera la derecha y frena la izquierda (corrige jalar a la izq).
    Devuelve (v_left_cal, v_right_cal).
    """
    return gain * v_left * (1.0 - trim), gain * v_right * (1.0 + trim)
