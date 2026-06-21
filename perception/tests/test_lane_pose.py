"""Tests de la pose de carril por proyeccion a tierra (error lateral metrico)."""
import numpy as np

from object_detection.ground_projection import estimate_homography, ground_to_pixel
from object_detection.lane_pose import lane_offset, estimate_lane_pose

HALF = 0.11  # media calzada del carril (m): linea a +-HALF cuando esta centrado

_H = estimate_homography(
    [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)],
    [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)],
)


# --- Geometria pura: offset a partir de las posiciones laterales de las lineas ---

def test_offset_centrado_es_cero():
    # amarilla a +HALF (izquierda), blanca a -HALF (derecha) -> centrado.
    assert abs(lane_offset(+HALF, -HALF, half_lane=HALF)) < 1e-9


def test_offset_derivado_a_la_derecha_es_positivo():
    # El robot derivo 0.05 a la derecha: ambas lineas se ven 0.05 mas a la izquierda.
    d = lane_offset(+HALF + 0.05, -HALF + 0.05, half_lane=HALF)
    assert abs(d - 0.05) < 1e-9


def test_offset_solo_amarilla():
    # Solo se ve la amarilla a +0.16 -> centro 0.05 a su derecha.
    assert abs(lane_offset(+0.16, None, half_lane=HALF) - 0.05) < 1e-9


def test_offset_solo_blanca():
    assert abs(lane_offset(None, -0.06, half_lane=HALF) - 0.05) < 1e-9


def test_offset_sin_lineas_es_none():
    assert lane_offset(None, None, half_lane=HALF) is None


# --- Pipeline real: imagen -> deteccion de lineas -> proyeccion -> d metrico ---

def _img_con_carril(centro_y):
    """Imagen sintetica con amarilla (izq) y blanca (der) en el suelo.

    `centro_y`: posicion lateral del centro del carril en el frame del robot (+izq).
    Pinta blobs amarillos/blancos en los pixeles donde proyectan los puntos de suelo.
    """
    img = np.full((480, 640, 3), (60, 60, 60), np.uint8)        # fondo gris
    for x in (0.14, 0.18, 0.22, 0.26, 0.30):                    # varias distancias
        for y, color in ((centro_y + HALF, (0, 200, 200)),       # amarilla (BGR)
                         (centro_y - HALF, (240, 240, 240))):     # blanca
            u, v = ground_to_pixel(_H, x, y)
            uu, vv = int(round(u)), int(round(v))
            img[max(0, vv - 8):vv + 8, max(0, uu - 8):uu + 8] = color
    return img


def test_estima_pose_centrado():
    r = estimate_lane_pose(_img_con_carril(0.0), _H, half_lane=HALF)
    assert r["detected"] is True
    assert abs(r["d"]) < 0.02                                    # ~centrado


def test_estima_pose_derivado_a_la_derecha():
    # centro del carril 0.05 a la izquierda del robot -> el robot derivo a la derecha.
    r = estimate_lane_pose(_img_con_carril(0.05), _H, half_lane=HALF)
    assert r["detected"] is True
    assert r["d"] > 0.02                                         # signo correcto
    assert abs(r["d"] - 0.05) < 0.03                             # magnitud razonable
