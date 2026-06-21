"""Tests de deteccion de la linea roja de stop."""
import numpy as np
from object_detection.stop_line import red_mask, detect_stop_line
from object_detection.anti_instagram import correct
from object_detection.ground_projection import estimate_homography


def _frame(h=100, w=60, color=(50, 50, 50)):
    """Frame BGR uniforme."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = color
    return img


def _con_banda_roja(fila0, fila1, h=100, w=60):
    """Frame gris con una banda roja horizontal entre fila0 y fila1."""
    img = _frame(h, w)
    img[fila0:fila1, :] = (0, 0, 255)        # rojo en BGR
    return img


def test_red_mask_detecta_rojo_y_no_otros():
    rojo = _frame(10, 10, color=(0, 0, 255))
    azul = _frame(10, 10, color=(255, 0, 0))
    assert np.count_nonzero(red_mask(rojo)) == 100      # todo rojo
    assert np.count_nonzero(red_mask(azul)) == 0        # nada de azul


def test_detecta_banda_roja_inferior():
    img = _con_banda_roja(80, 95)            # banda abajo (frente del robot)
    r = detect_stop_line(img)
    assert r["detected"] is True
    assert r["red_fraction"] > 0
    assert 0.0 <= r["distance"] <= 1.0


def test_sin_rojo_no_detecta():
    r = detect_stop_line(_frame())
    assert r["detected"] is False
    assert r["distance"] is None
    assert r["centroid_row"] is None


def test_banda_azul_no_dispara():
    img = _frame()
    img[80:95, :] = (255, 0, 0)              # azul, no rojo
    assert detect_stop_line(img)["detected"] is False


def test_mas_abajo_es_mas_cerca():
    cerca = detect_stop_line(_con_banda_roja(90, 99))    # casi en el borde inferior
    lejos = detect_stop_line(_con_banda_roja(52, 60))    # mas arriba (dentro de ROI)
    assert cerca["detected"] and lejos["detected"]
    assert cerca["distance"] < lejos["distance"]


def test_umbral_de_fraccion():
    # una pizca de rojo (1 fila de 100) en la ROI no debe disparar con umbral alto
    img = _con_banda_roja(98, 99)
    assert detect_stop_line(img, min_fraction=0.2)["detected"] is False
    assert detect_stop_line(img, min_fraction=0.0001)["detected"] is True


def test_roi_ignora_rojo_superior():
    # rojo SOLO en la mitad superior -> con ROI inferior (0.5) no se considera
    img = _frame()
    img[5:20, :] = (0, 0, 255)
    assert detect_stop_line(img, roi_frac=0.5)["detected"] is False
    # pero si miramos toda la imagen, si
    assert detect_stop_line(img, roi_frac=1.0)["detected"] is True


def test_distancia_metrica_con_homografia():
    # Con homografia, detect_stop_line devuelve distance_m REAL (metros), no solo
    # el proxy [0..1]: una franja mas abajo en la imagen -> menor distancia metrica.
    H = estimate_homography(
        [(180.0, 400.0), (460.0, 400.0), (260.0, 250.0), (380.0, 250.0)],
        [(0.10, 0.10), (0.10, -0.10), (0.30, 0.10), (0.30, -0.10)],
    )
    img_h, img_w = 480, 640
    cerca = np.full((img_h, img_w, 3), (50, 50, 50), np.uint8)
    cerca[400:440, :] = (0, 0, 255)              # franja abajo (cerca)
    lejos = np.full((img_h, img_w, 3), (50, 50, 50), np.uint8)
    lejos[250:290, :] = (0, 0, 255)              # franja mas arriba (lejos)

    r_cerca = detect_stop_line(cerca, homography=H)
    r_lejos = detect_stop_line(lejos, homography=H, roi_frac=1.0)
    assert r_cerca["distance_m"] is not None
    assert r_cerca["distance_m"] < r_lejos["distance_m"]   # mas abajo = mas cerca
    assert 0.0 < r_cerca["distance_m"] < 1.0               # metros, escala razonable

    # Sin homografia: distance_m es None (compatibilidad hacia atras).
    assert detect_stop_line(cerca)["distance_m"] is None


def test_sobreexpuesta_recuperable_tras_anti_instagram():
    # Sobreexposicion fuerte: la franja roja se lava a (210,210,255) sobre piso
    # blanco. El rojo crudo cae bajo el umbral de saturacion, pero queda info
    # recuperable (canales B/G). Tras anti-Instagram debe volver a detectarse.
    img = np.full((100, 60, 3), (120, 120, 120), np.uint8)
    img[70:95, :] = (40, 40, 200)                        # franja roja
    over = np.clip(img.astype(np.int32) + 170, 0, 255).astype(np.uint8)
    assert detect_stop_line(over)["detected"] is False    # cruda: se pierde
    assert detect_stop_line(correct(over))["detected"] is True  # corregida: vuelve
