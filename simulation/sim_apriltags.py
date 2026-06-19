"""Simulacion y validacion de deteccion de AprilTags (las "senaleticas QR").

Genera un AprilTag real, lo detecta con pupil-apriltags, y mapea el tag_id a
su accion via la tabla oficial (apriltag_policy). Valida la cadena completa
generar -> detectar -> accion SIN robot ni camara.

Nota de familia: para poder GENERAR localmente se usa `tagStandard41h12`
(soportada por moms-apriltag y detectable por pupil-apriltags). El robot real
usa `tag36h11` (Duckietown); el mapeo tag_id -> senal -> accion es identico,
solo cambia la codificacion del marcador.
"""
import numpy as np

FAMILY_SIM = "tagStandard41h12"   # familia generable+detectable en la Mac (simulacion)
FAMILY_ROBOT = "tag36h11"         # familia real de Duckietown (en el robot)


def make_tag_image(tag_id, family=FAMILY_SIM, scale=24, border_cells=2):
    """Genera la imagen (gris, uint8) de un AprilTag con borde blanco (quiet zone)."""
    import moms_apriltag as apt
    tag = np.asarray(apt.TagGenerator3(family).generate(tag_id))
    tag = np.where(tag > 0, 255, 0).astype(np.uint8)          # binarizar a 0/255
    big = np.kron(tag, np.ones((scale, scale), dtype=np.uint8))
    pad = border_cells * scale
    img = np.full((big.shape[0] + 2 * pad, big.shape[1] + 2 * pad), 255, dtype=np.uint8)
    img[pad:pad + big.shape[0], pad:pad + big.shape[1]] = big
    return img


def detect_tag_ids(img, family=FAMILY_SIM):
    """Detecta AprilTags en una imagen y devuelve la lista de tag_id."""
    from pupil_apriltags import Detector
    arr = np.asarray(img)
    if arr.ndim == 3:
        import cv2
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    arr = np.ascontiguousarray(arr, dtype=np.uint8)
    det = Detector(families=family)
    return [r.tag_id for r in det.detect(arr)]


def detect_and_act(img, family=FAMILY_SIM):
    """Detecta tags y devuelve (lista_ids, accion) usando la politica oficial."""
    from object_detection.apriltag_policy import select_action
    ids = detect_tag_ids(img, family)
    return ids, select_action(ids)


def decide_with_signs(tag_ids, planned_turn=None):
    """Combina las senales detectadas con el giro planeado (enlace con planificacion).

    Una senal de seguridad (stop / yield) MANDA sobre el plan global; si no hay
    senal accionable, el robot sigue el giro planeado por el route_planner.
    """
    from object_detection.apriltag_policy import select_action
    accion = select_action(tag_ids)
    if accion in ("stop", "yield"):
        return accion
    return planned_turn
