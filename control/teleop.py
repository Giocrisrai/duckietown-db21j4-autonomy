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


def _demo(argv):
    """Demo CLI del mapeo manual: throttle steer [max_speed] -> ruedas.
    Uso: python -m teleop 0.6 0.2   (sin args muestra una tabla de ejemplos)."""
    if len(argv) >= 2:
        thr, st = float(argv[0]), float(argv[1])
        mx = float(argv[2]) if len(argv) > 2 else 0.5
        vl, vr = joystick_to_wheels(thr, st, mx)
        print(f"throttle={thr:+.2f} steer={st:+.2f} -> v_left={vl:+.3f} v_right={vr:+.3f}")
        return
    print("Mapeo manual joystick/teclado -> ruedas (v_left, v_right):")
    for thr, st in [(1, 0), (-1, 0), (0, 1), (0, -1), (0.6, 0.2)]:
        vl, vr = joystick_to_wheels(thr, st)
        print(f"  throttle={thr:+.1f} steer={st:+.1f} -> ({vl:+.3f}, {vr:+.3f})")


if __name__ == "__main__":
    import sys
    _demo(sys.argv[1:])
