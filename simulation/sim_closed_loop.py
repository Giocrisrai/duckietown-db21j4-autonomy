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

Geometria: marco de Frenet (curvilineo) sobre el carril. El robot tiene pose
(off, dpsi) = desviacion lateral respecto al centro (m, +izquierda) y error de
rumbo respecto a la tangente del carril (rad). El carril tiene curvatura `curvature`
(1/radio, >0 = gira a la izquierda; 0 = recto). HALF_LANE es la media calzada.
"""
import math

import numpy as np

from object_detection.ground_projection import estimate_homography, ground_to_pixel
from object_detection.lane_pose import HALF_LANE
from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.fsm import STOPPED
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


def render(off, dpsi, curvature=0.0):
    """Renderiza la imagen de la camara para una pose (off, dpsi) y curvatura dada.

    Muestrea el centro del carril como un arco de curvatura constante a varias
    distancias, lo lleva al frame del robot y desplaza +-HALF por la normal
    izquierda del carril para las lineas amarilla (izq) y blanca (der).
    """
    img = np.full((IMG_H, IMG_W, 3), BG_BGR, np.uint8)
    cd, sd = math.cos(dpsi), math.sin(dpsi)
    for l in _ARCS:
        if abs(curvature) < 1e-9:                   # recto
            x_l, y_l, tang = l, 0.0, 0.0
        else:                                       # arco: punto a arco-longitud l
            x_l = math.sin(curvature * l) / curvature
            y_l = (1.0 - math.cos(curvature * l)) / curvature
            tang = curvature * l                    # giro de la tangente del carril en l
        dx, dy = x_l, y_l - off                      # vector al punto en el frame del carril
        x_r = cd * dx + sd * dy                      # rotar -dpsi al frame del robot
        y_r = -sd * dx + cd * dy
        normal = tang - dpsi + math.pi / 2.0         # normal izquierda del carril en el robot
        nx, ny = math.cos(normal), math.sin(normal)
        _paint(img, x_r + HALF_LANE * nx, y_r + HALF_LANE * ny, YELLOW_BGR)  # amarilla (izq)
        _paint(img, x_r - HALF_LANE * nx, y_r - HALF_LANE * ny, WHITE_BGR)   # blanca (der)
    return img


def _perturb(img, rng, noise_sigma, brightness, occlusion):
    """Aplica ruido realista de camara: gaussiano, sesgo de brillo y oclusion.

    occlusion: fraccion (0..1) del ancho tapada por una banda vertical (sombra/pato).
    """
    out = img.astype(np.int16)
    if brightness:
        out += int(brightness)
    if noise_sigma > 0:
        out += rng.normal(0, noise_sigma, img.shape).astype(np.int16)
    out = np.clip(out, 0, 255).astype(np.uint8)
    if occlusion > 0:
        w = int(IMG_W * occlusion)
        x0 = (IMG_W - w) // 2
        out[:, x0:x0 + w] = BG_BGR                  # banda central tapada
    return out


_OBST_RANGE = 0.9        # m: distancia a la que el obstaculo del mapa entra en vision
_LAT_PX = IMG_W * 0.40 / 0.10  # px por metro lateral (parallax: al apartarse, el obstaculo cercano se corre)


def _obstacle_detection(s, off, obstacle):
    """Deteccion sintetica del obstaculo del mapa segun la pose del robot, o None.

    obstacle = (s_obs, y_obs): distancia longitudinal (m) y posicion lateral (m,
    +izquierda) del obstaculo. El bbox refleja su cercania (mas bajo/abajo = mas
    cerca) y su posicion lateral aparente (parallax al apartarse el robot)."""
    s_obs, y_obs = obstacle
    d_long = s_obs - s
    if not (0.0 < d_long <= _OBST_RANGE):
        return None
    prox = 1.0 - d_long / _OBST_RANGE                # cerca -> ~1
    y2 = max(1.0, prox * IMG_H)
    cx = IMG_W / 2.0 - (y_obs - off) * _LAT_PX       # +izquierda -> x menor
    return {"class_name": "duckie", "confidence": 0.9,
            "bbox": (cx - 45, y2 - 90, cx + 45, y2)}


def run_image_loop(off0=0.06, psi0=0.0, steps=200, dt=0.05, base_speed=0.3,
                   kp=3.0, k_heading=0.5, curvature=0.0,
                   noise_sigma=0.0, brightness=0, occlusion=0.0, seed=0,
                   obstacle=None, avoid_offset=0.0):
    """Corre el lazo cerrado imagen->imagen y devuelve un trace.

    Control PROPORCIONAL puro sobre el error de carril combinado d + k_heading*phi:
    el termino de rumbo ya aporta el amortiguamiento (es accion derivativa fisica),
    asi que no hace falta un kd que oscilaria con el ruido de phi.

    curvature: curvatura del carril (1/m, >0 = curva a la izquierda; 0 = recto).
    obstacle: (s_obs, y_obs) de un obstaculo del mapa en la pista; con avoid_offset>0
    el robot lo RODEA (se aparta y vuelve al centro tras rebasarlo).

    Devuelve dict: off, dpsi, omega, s (avance), stopped (si freno por el obstaculo),
    max_abs_off, converged.
    """
    pid = PID(kp, 0.0, 0.0, out_limits=(-5.0, 5.0))
    ctrl = AutonomousController(homography=_H, base_speed=base_speed,
                               img_size=(IMG_W, IMG_H), pid=pid, k_heading=k_heading,
                               avoid_offset=avoid_offset)
    rng = np.random.default_rng(seed)
    off, dpsi, s = float(off0), float(psi0), 0.0
    offs, dpsis, omegas = [off], [dpsi], []
    stopped = False
    for _ in range(steps):
        img = render(off, dpsi, curvature)
        if noise_sigma or brightness or occlusion:
            img = _perturb(img, rng, noise_sigma, brightness, occlusion)
        det = _obstacle_detection(s, off, obstacle) if obstacle else None
        cmd = ctrl.step({"image": img, "img_w": IMG_W, "img_h": IMG_H,
                         "detections": [det] if det else []}, dt)
        v, omega = cmd["v"], cmd["omega"]
        if cmd["state"] == STOPPED:
            stopped = True
        # Modelo cinematico en el marco de Frenet (carril de curvatura `curvature`):
        denom = max(1.0 - off * curvature, 1e-3)     # evita singularidad si off*k -> 1
        sdot = v * math.cos(dpsi) / denom            # avance a lo largo del centro
        s += sdot * dt
        off += v * math.sin(dpsi) * dt
        dpsi += (omega - curvature * sdot) * dt       # el rumbo del carril tambien rota
        offs.append(off)
        dpsis.append(dpsi)
        omegas.append(omega)
    return {"off": offs, "dpsi": dpsis, "omega": omegas, "s": s, "stopped": stopped,
            "max_abs_off": max(abs(o) for o in offs),
            "converged": abs(offs[-1]) < 0.03}


def _demo():
    print("== Lazo cerrado IMAGEN->IMAGEN: seguir-carril (el cerebro solo ve pixeles) ==")
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200)
    print("  [recta]  desviacion inicial: {:+.3f} m -> final: {:+.3f} m  (pico {:.3f})".format(
        r["off"][0], r["off"][-1], r["max_abs_off"]))
    rc = run_image_loop(off0=0.0, psi0=0.0, steps=200, curvature=1.2)
    media_omega = sum(rc["omega"][len(rc["omega"]) // 2:]) / (len(rc["omega"]) // 2)
    print("  [curva]  R={:.2f} m: error lateral max {:.3f} m, omega medio {:+.2f} rad/s (sigue la curva)".format(
        1.0 / 1.2, rc["max_abs_off"], media_omega))
    rn = run_image_loop(off0=0.06, psi0=0.10, steps=200, noise_sigma=25,
                        occlusion=0.12, seed=1)
    print("  [ruido]  sigma=25 + oclusion 12%: desviacion final {:+.3f} m (robusto)".format(
        rn["off"][-1]))
    rsin = run_image_loop(off0=0.0, psi0=0.0, steps=500, obstacle=(1.0, 0.0), avoid_offset=0.0)
    ro = run_image_loop(off0=0.0, psi0=0.0, steps=500, obstacle=(1.0, 0.0), avoid_offset=0.22)
    print("  [esquive] obstaculo en pista: SIN esquive avanza {:.2f} m (atascado); "
          "CON esquive rodea (pico {:+.3f} m) y avanza {:.2f} m".format(
              rsin["s"], max(ro["off"]), ro["s"]))
    print("  OK: el lazo percepcion->control siguio el carril (recto, curva, ruido) y "
          "esquivo el obstaculo del mapa."
          if r["converged"] and rc["max_abs_off"] < 0.08 and abs(rn["off"][-1]) < 0.03
          and ro["s"] > 1.0 else "  FALLO.")


if __name__ == "__main__":
    _demo()
