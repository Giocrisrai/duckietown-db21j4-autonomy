"""Fusion de obstaculos VISUAL (YOLO26) + ToF (paridad con dt-core obstacle_detection).

El oficial hace un OR entre la deteccion visual de vehiculo/peaton y el rango del
ToF para generar una "virtual stop line". Nosotros tenemos un detector mucho mejor
(YOLO26 -> patos/conos/vehiculos), asi que ademas de frenar en seco con el ToF a
15 cm, podemos **anticipar**: si vemos un pato/coche en el camino y cerca, frenamos
suave ANTES de que el ToF dispare.

Entrada de vision: lista de detecciones, cada una un dict con al menos
`class_name` y `bbox` = (x1, y1, x2, y2) en pixeles. La cercania es un proxy por la
posicion del borde inferior del bbox (mas abajo en la imagen = mas cerca); para
distancia metrica real haria falta la calibracion extrinseca.
"""
from . import safety

# Clases de YOLO26 que son obstaculos a evitar (el resto se ignora para frenar).
OBSTACLE_CLASSES = ("duckie", "duckiebot", "cone", "truck", "pedestrian")

SLOW_PROX = 0.50      # proximidad (0..1) a partir de la cual se frena suave
STOP_PROX = 0.80      # proximidad a partir de la cual se para
PATH_BAND = 0.50      # ancho (fraccion) del corredor central que cuenta como "en el camino"
TOF_THRESHOLD = 0.15  # m: freno en seco del ToF
MIN_CONF = 0.25       # confianza minima de la deteccion para considerarla

# Umbrales METRICOS (m) cuando hay homografia (calibracion extrinseca): frenar suave
# al acercarse y parar en seco. Mas principiados que el proxy de fila del bbox.
SLOW_DIST_M = 0.40    # distancia (m) a partir de la cual se frena suave
STOP_DIST_M = 0.20    # distancia (m) a partir de la cual se para
_W_EPS = 1e-9         # denominador homogeneo ~0 -> pixel en el horizonte (se ignora)


def bbox_proximity(bbox, img_h):
    """Proxy de cercania en [0..1] segun el borde INFERIOR del bbox (y2/img_h)."""
    _, _, _, y2 = bbox
    return max(0.0, min(1.0, float(y2) / float(img_h)))


def in_path(bbox, img_w, band=PATH_BAND):
    """True si el bbox solapa el corredor central de ancho `band` (fraccion)."""
    x1, _, x2, _ = bbox
    lo = (0.5 - band / 2.0) * img_w
    hi = (0.5 + band / 2.0) * img_w
    return not (x2 < lo or x1 > hi)            # hay solape horizontal con el corredor


def _vision_proximity(detections, img_w, img_h, classes, band, min_conf):
    """Maxima proximidad entre los obstaculos relevantes que estan en el camino."""
    best = 0.0
    for d in detections or []:
        if d.get("class_name") not in classes:
            continue
        if d.get("confidence", 1.0) < min_conf:
            continue
        bbox = d.get("bbox")
        if bbox is None or not in_path(bbox, img_w, band):
            continue
        best = max(best, bbox_proximity(bbox, img_h))
    return best


def _forward_distance(homography, u, v):
    """Distancia hacia adelante (m) del suelo bajo el pixel (u, v), o None si el
    pixel cae en el horizonte / detras del robot (no es un obstaculo en el camino)."""
    h = homography
    X = h[0][0] * u + h[0][1] * v + h[0][2]
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    if abs(w) < _W_EPS:
        return None
    x = X / w
    return x if x > 0 else None


def _vision_metric_distance(detections, homography, img_w, img_h, classes, band, min_conf):
    """Menor distancia metrica (m) a un obstaculo relevante en el camino, o None.

    Proyecta el centro del borde INFERIOR del bbox (el punto que toca el suelo)
    al plano del suelo. Es mas fiel que el proxy de fila: la distancia real no
    depende del tamano aparente del objeto.
    """
    best = None
    for d in detections or []:
        if d.get("class_name") not in classes:
            continue
        if d.get("confidence", 1.0) < min_conf:
            continue
        bbox = d.get("bbox")
        if bbox is None or not in_path(bbox, img_w, band):
            continue
        x1, _, x2, y2 = bbox
        dist = _forward_distance(homography, (x1 + x2) / 2.0, y2)
        if dist is not None and (best is None or dist < best):
            best = dist
    return best


def fuse(detections, tof_distance_m, img_w, img_h,
         classes=OBSTACLE_CLASSES, slow_prox=SLOW_PROX, stop_prox=STOP_PROX,
         band=PATH_BAND, tof_threshold=TOF_THRESHOLD, min_conf=MIN_CONF,
         homography=None, slow_dist_m=SLOW_DIST_M, stop_dist_m=STOP_DIST_M):
    """Combina vision + ToF en una decision de frenado.

    Sin `homography`: la cercania visual es el proxy de fila del bbox (0..1).
    Con `homography` (calibracion extrinseca): se usa la distancia METRICA real
    al suelo y los umbrales en metros (slow_dist_m / stop_dist_m), mas principiada.

    Devuelve un dict:
      - action: "GO" | "SLOW" | "STOP"
      - brake: 0.0 (libre) .. 1.0 (freno total)
      - reason: "tof" | "vision" | "tof+vision" | "clear"
      - tof_stop (bool), max_proximity (float)
      - distance_m (float|None): distancia metrica al obstaculo en el camino
        (solo con homografia; None en otro caso).
    """
    tof_stop = safety.should_brake(tof_distance_m, tof_threshold)
    prox = _vision_proximity(detections, img_w, img_h, classes, band, min_conf)

    if homography is not None:
        dist_m = _vision_metric_distance(
            detections, homography, img_w, img_h, classes, band, min_conf)
        vision_stop = dist_m is not None and dist_m <= stop_dist_m
        vision_slow = dist_m is not None and stop_dist_m < dist_m <= slow_dist_m
        # frenado graduado: cerca de stop_dist_m -> 1.0; cerca de slow_dist_m -> 0.3
        slow_frac = (slow_dist_m - dist_m) / max(slow_dist_m - stop_dist_m, 1e-6) \
            if vision_slow else 0.0
    else:
        dist_m = None
        vision_stop = prox >= stop_prox
        vision_slow = slow_prox <= prox < stop_prox
        slow_frac = (prox - slow_prox) / max(stop_prox - slow_prox, 1e-6) \
            if vision_slow else 0.0

    if tof_stop or vision_stop:
        reason = "tof+vision" if (tof_stop and vision_stop) else ("tof" if tof_stop else "vision")
        return {"action": "STOP", "brake": 1.0, "reason": reason,
                "tof_stop": tof_stop, "max_proximity": prox, "distance_m": dist_m}
    if vision_slow:
        brake = 0.3 + slow_frac * 0.7         # brake en [0.3, 1.0)
        return {"action": "SLOW", "brake": round(brake, 3), "reason": "vision",
                "tof_stop": False, "max_proximity": prox, "distance_m": dist_m}
    return {"action": "GO", "brake": 0.0, "reason": "clear",
            "tof_stop": False, "max_proximity": prox, "distance_m": dist_m}
