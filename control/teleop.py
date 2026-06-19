"""Teleoperacion (joystick/teclado) -> velocidades de rueda diferenciales.

Mapea entradas de throttle/steer a (v_left, v_right). Reemplaza/funciona como el
joy_mapper oficial; la salida se puede pasar por la calibracion (apply_trim_gain).
"""

def joystick_to_wheels(throttle, steer, max_speed=0.5):
    """Throttle y steer en [-1,1] -> (v_left, v_right) en [-max_speed, max_speed].

    throttle=+1 avanza; steer=+1 gira a la derecha (rueda izq mas rapida).
    """
    throttle = max(-1.0, min(1.0, throttle))
    steer = max(-1.0, min(1.0, steer))
    v = throttle * max_speed
    vl = v + steer * max_speed
    vr = v - steer * max_speed
    clamp = lambda x: max(-max_speed, min(max_speed, x))
    return clamp(vl), clamp(vr)
