"""Tests de la pose de carril por proyeccion a tierra (error lateral metrico)."""
import math
import numpy as np

from object_detection.ground_projection import estimate_homography, ground_to_pixel
from object_detection.lane_pose import (
    lane_offset, lane_heading, steering_error, estimate_lane_pose,
)

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


# --- Geometria pura del rumbo (phi) a partir de las pendientes de las lineas ---

def test_heading_recto_es_cero():
    assert abs(lane_heading(0.0, 0.0)) < 1e-9


def test_heading_pendiente_da_angulo():
    # pendiente dy/dx = tan(0.2) en ambas lineas -> phi ~ 0.2 rad.
    m = math.tan(0.2)
    assert abs(lane_heading(m, m) - 0.2) < 1e-9


def test_heading_una_sola_linea():
    m = math.tan(0.15)
    assert abs(lane_heading(m, None) - 0.15) < 1e-9


def test_heading_sin_lineas_es_none():
    assert lane_heading(None, None) is None


def test_steering_error_combina_d_y_phi():
    # error de direccion = d + k*phi (cross-track + heading); None-safe.
    assert abs(steering_error(0.04, 0.2, k_heading=0.5) - (0.04 + 0.1)) < 1e-9
    assert steering_error(None, None) is None


# --- Pipeline real: imagen -> deteccion de lineas -> proyeccion -> (d, phi) ---

def _img_con_carril(centro_y, pendiente=0.0):
    """Imagen sintetica con amarilla (izq) y blanca (der) en el suelo.

    `centro_y`: posicion lateral del centro del carril a x=0 (+izq).
    `pendiente`: dy/dx del carril (0 = recto delante del robot).
    Pinta blobs amarillos/blancos en los pixeles donde proyectan los puntos de suelo.
    """
    img = np.full((480, 640, 3), (60, 60, 60), np.uint8)        # fondo gris
    for x in (0.14, 0.18, 0.22, 0.26, 0.30):                    # varias distancias
        cy = centro_y + pendiente * x
        for y, color in ((cy + HALF, (0, 200, 200)),             # amarilla (BGR)
                         (cy - HALF, (240, 240, 240))):          # blanca
            u, v = ground_to_pixel(_H, x, y)
            uu, vv = int(round(u)), int(round(v))
            img[max(0, vv - 8):vv + 8, max(0, uu - 8):uu + 8] = color
    return img


def test_estima_pose_centrado():
    r = estimate_lane_pose(_img_con_carril(0.0), _H, half_lane=HALF)
    assert r["detected"] is True
    assert abs(r["d"]) < 0.02                                    # ~centrado
    assert abs(r["phi"]) < 0.05                                  # recto


def test_estima_pose_derivado_a_la_derecha():
    # centro del carril 0.05 a la izquierda del robot -> el robot derivo a la derecha.
    r = estimate_lane_pose(_img_con_carril(0.05), _H, half_lane=HALF)
    assert r["detected"] is True
    assert r["d"] > 0.02                                         # signo correcto
    assert abs(r["d"] - 0.05) < 0.03                             # magnitud razonable


def test_estima_rumbo_phi():
    # Carril que se va a la izquierda al alejarse (pendiente>0) -> phi>0.
    r = estimate_lane_pose(_img_con_carril(0.0, pendiente=math.tan(0.25)), _H, half_lane=HALF)
    assert r["phi"] > 0.05
    assert abs(r["phi"] - 0.25) < 0.08                           # magnitud razonable


def _blobs_supervivientes(img, n, seed):
    # Manchas de color de linea de 4px (sobreviven la apertura 3x3) dispersas en la ROI.
    rng = np.random.default_rng(seed)
    for _ in range(n):
        x, y = int(rng.integers(20, 620)), int(rng.integers(250, 470))
        col = (0, 200, 200) if rng.random() < 0.5 else (240, 240, 240)
        img[max(0, y - 2):y + 2, max(0, x - 2):x + 2] = col
    return img


def test_speckle_superviviente_no_sesga_la_pose():
    # Manchas espurias de color de linea que SOBREVIVEN la apertura y llegan al
    # estimador: la mediana robusta las rechaza y la pose se mantiene ~centrada.
    # (Un estimador por MEDIA se sesgaria; la mediana no.)
    for seed in range(4):
        img = _blobs_supervivientes(_img_con_carril(0.0), 40, seed)
        r = estimate_lane_pose(img, _H, half_lane=HALF)
        assert r["detected"] is True
        assert abs(r["d"]) < 0.03            # sigue ~centrado pese al ruido estructurado
