"""Tests de la fusion de obstaculos visual (YOLO26) + ToF."""
from duckietown_planning.obstacle_fusion import (
    fuse, in_path, bbox_proximity, OBSTACLE_CLASSES,
)

W, H = 640, 480


def _det(class_name, bbox, confidence=0.9):
    return {"class_name": class_name, "bbox": bbox, "confidence": confidence}


def _centro(y2, alto=120, ancho=120):
    """bbox centrado horizontalmente con borde inferior en y2."""
    cx = W / 2
    return (cx - ancho / 2, y2 - alto, cx + ancho / 2, y2)


def test_bbox_proximity_borde_inferior():
    assert bbox_proximity((0, 0, 10, H), H) == 1.0          # toca el fondo
    assert bbox_proximity((0, 0, 10, H // 2), H) == 0.5


def test_in_path_centro_vs_lateral():
    assert in_path(_centro(H), W) is True
    assert in_path((0, 0, 30, 30), W) is False              # pegado a la izquierda


def test_tof_cercano_para_sin_vision():
    r = fuse([], 0.10, W, H)
    assert r["action"] == "STOP" and r["reason"] == "tof"


def test_tof_lejano_sin_obstaculos_avanza():
    r = fuse([], 1.0, W, H)
    assert r["action"] == "GO" and r["brake"] == 0.0


def test_pato_cerca_en_camino_para_por_vision():
    det = [_det("duckie", _centro(int(0.9 * H)))]            # bajo en la imagen -> cerca
    r = fuse(det, 1.0, W, H)
    assert r["action"] == "STOP" and r["reason"] == "vision"


def test_pato_distancia_media_frena_suave():
    det = [_det("duckie", _centro(int(0.6 * H)))]            # proximidad ~0.6 -> SLOW
    r = fuse(det, 1.0, W, H)
    assert r["action"] == "SLOW"
    assert 0.0 < r["brake"] < 1.0


def test_pato_lejos_avanza():
    det = [_det("duckie", _centro(int(0.3 * H)))]            # alto en la imagen -> lejos
    assert fuse(det, 1.0, W, H)["action"] == "GO"


def test_obstaculo_fuera_del_camino_no_frena():
    det = [_det("duckie", (0, int(0.85 * H), 60, int(0.98 * H)))]   # cerca pero al lado
    assert fuse(det, 1.0, W, H)["action"] == "GO"


def test_clase_irrelevante_se_ignora():
    det = [_det("sign", _centro(int(0.95 * H)))]            # 'sign' no es obstaculo
    assert "sign" not in OBSTACLE_CLASSES
    assert fuse(det, 1.0, W, H)["action"] == "GO"


def test_baja_confianza_se_ignora():
    det = [_det("duckie", _centro(int(0.95 * H)), confidence=0.1)]
    assert fuse(det, 1.0, W, H)["action"] == "GO"


def test_tof_y_vision_juntos_reportan_ambos():
    det = [_det("duckiebot", _centro(int(0.9 * H)))]
    r = fuse(det, 0.10, W, H)
    assert r["action"] == "STOP" and r["reason"] == "tof+vision"


def test_frenado_graduado_monotono():
    cerca = fuse([_det("duckie", _centro(int(0.75 * H)))], 1.0, W, H)["brake"]
    lejos = fuse([_det("duckie", _centro(int(0.55 * H)))], 1.0, W, H)["brake"]
    assert cerca > lejos                                    # mas cerca -> mas freno
