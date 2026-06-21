"""Lazo cerrado IMAGEN->IMAGEN del seguir-carril (sin robot, sin escalares privilegiados).

Es la validacion mas honesta del stack de percepcion+control: en cada tick
  1) el MUNDO renderiza la camara desde la pose del robot (lineas amarilla/blanca
     proyectadas a pixeles con la homografia),
  2) el CEREBRO (AutonomousController) recibe SOLO esa imagen, estima la pose de
     carril (d, phi) por proyeccion a tierra y comanda (v, omega),
  3) el MODELO cinematico de triciclo integra la pose con ese comando.

No se le pasa al cerebro ningun error ya calculado: si algun signo de la cadena
(proyeccion, d, phi, PID) estuviese mal, el lazo divergiria. La convergencia al
centro valida toda la cadena de una vez.

Geometria: mundo con el carril recto a lo largo de X (centro en Y=0). El robot
tiene pose (s, off, theta) = avance, desviacion lateral (m, +izquierda) y rumbo
absoluto (rad, 0 = alineado con el carril). HALF_LANE es la media calzada.
"""
import math

import numpy as np

from object_detection.ground_projection import estimate_homography, ground_to_pixel
from object_detection.lane_pose import HALF_LANE
from duckietown_planning.autonomy import AutonomousController
from pid import PID

IMG_W, IMG_H = 640, 480
YELLOW_BGR = (0, 200, 200)
WHITE_BGR = (240, 240, 240)
BG_BGR = (60, 60, 60)

# Homografia de referencia (la misma calibracion sintetica usada en los tests).
_H = estimate_homography(
    [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)],
    [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)],
)
# Distancias de avance (m) a las que se muestrean las lineas visibles delante.
_ARCS = (0.13, 0.17, 0.21, 0.25, 0.29)


def _paint(img, x_r, y_r, color, blob=7):
    """Pinta un blob en el pixel donde proyecta el punto de suelo (x_r, y_r)."""
    if x_r <= 0:
        return
    u, v = ground_to_pixel(_H, x_r, y_r)
    uu, vv = int(round(u)), int(round(v))
    if 0 <= uu < IMG_W and 0 <= vv < IMG_H:
        img[max(0, vv - blob):vv + blob, max(0, uu - blob):uu + blob] = color


def render(off, theta):
    """Renderiza la imagen de la camara para una pose (off, theta) del robot.

    Coloca puntos del centro del carril a varias distancias y los desplaza +-HALF
    por la normal izquierda para las lineas amarilla (izq) y blanca (der).
    """
    img = np.full((IMG_H, IMG_W, 3), BG_BGR, np.uint8)
    c, s = math.cos(theta), math.sin(theta)
    nx, ny = s, c                                  # normal izquierda del carril en frame robot
    for l in _ARCS:
        x_c = l * c - off * s                       # centro del carril en frame robot
        y_c = -l * s - off * c
        _paint(img, x_c + HALF_LANE * nx, y_c + HALF_LANE * ny, YELLOW_BGR)  # amarilla (izq)
        _paint(img, x_c - HALF_LANE * nx, y_c - HALF_LANE * ny, WHITE_BGR)   # blanca (der)
    return img


def run_image_loop(off0=0.06, psi0=0.0, steps=200, dt=0.05, base_speed=0.3,
                   kp=3.0, k_heading=0.5):
    """Corre el lazo cerrado imagen->imagen y devuelve un trace.

    Control PROPORCIONAL puro sobre el error de carril combinado d + k_heading*phi:
    el termino de rumbo ya aporta el amortiguamiento (es accion derivativa fisica),
    asi que no hace falta un kd que oscilaria con el ruido de phi.

    Devuelve dict: off (lista de desviaciones laterales), theta (lista de rumbos),
    max_abs_off, converged (|off| final < 2 cm).
    """
    pid = PID(kp, 0.0, 0.0, out_limits=(-5.0, 5.0))
    ctrl = AutonomousController(homography=_H, base_speed=base_speed,
                               img_size=(IMG_W, IMG_H), pid=pid, k_heading=k_heading)
    off, theta = float(off0), float(psi0)
    offs, thetas = [off], [theta]
    for _ in range(steps):
        img = render(off, theta)
        cmd = ctrl.step({"image": img, "img_w": IMG_W, "img_h": IMG_H}, dt)
        v, omega = cmd["v"], cmd["omega"]
        # Modelo cinematico de triciclo (carril recto en Y=0):
        theta += omega * dt
        off += v * math.sin(theta) * dt
        offs.append(off)
        thetas.append(theta)
    return {"off": offs, "theta": thetas,
            "max_abs_off": max(abs(o) for o in offs),
            "converged": abs(offs[-1]) < 0.02}


def _demo():
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200)
    print("== Lazo cerrado IMAGEN->IMAGEN: seguir-carril (el cerebro solo ve pixeles) ==")
    print(f"  desviacion inicial : {r['off'][0]:+.3f} m  (rumbo {math.degrees(r['theta'][0]):+.1f} deg)")
    print(f"  desviacion final   : {r['off'][-1]:+.3f} m")
    print(f"  pico transitorio   : {r['max_abs_off']:.3f} m")
    print("  OK: el lazo percepcion->control convergio al centro del carril."
          if r["converged"] else "  FALLO: no convergio.")


if __name__ == "__main__":
    _demo()
