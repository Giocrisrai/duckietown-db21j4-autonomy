"""Anti-Instagram: normalizacion de color/iluminacion (paridad con dt-core).

El detector de linea CLASICO (que reutilizamos de dt-core) umbraliza en HSV, y
esos umbrales se rompen cuando cambia la luz del recinto (sombras, luz calida/fria,
sobre/sub-exposicion). Anti-Instagram corrige esto ANTES de detectar, aplicando una
transformacion lineal por canal que lleva el rango real de la imagen a [0, 255]
(estiramiento robusto por percentiles = balance de blancos + normalizacion de
contraste). Es barato y de alto impacto en robustez.

Uso tipico:
    corregida = correct(frame_bgr)        # luego -> line_detector
o, para reusar la misma transformacion en un video (mas estable y barato):
    scale, shift = compute_transform(frame_referencia)
    corregida = apply_transform(frame, scale, shift)
"""
import numpy as np

# Percentiles robustos por defecto: ignora el 1% extremo (ruido/reflejos especulares).
LOW_PCT = 1.0
HIGH_PCT = 99.0
EPS = 1e-6


def channel_bounds(img, low_pct=LOW_PCT, high_pct=HIGH_PCT):
    """Devuelve (low[C], high[C]): percentiles por canal de una imagen HxWxC.

    Son los limites robustos del rango real de cada canal (BGR).
    """
    img = np.asarray(img, dtype=np.float64)
    if img.ndim != 3:
        raise ValueError("se espera una imagen HxWxC (3 canales)")
    if img.size == 0:
        raise ValueError("imagen vacia")
    c = img.shape[2]
    low = np.array([np.percentile(img[:, :, i], low_pct) for i in range(c)])
    high = np.array([np.percentile(img[:, :, i], high_pct) for i in range(c)])
    return low, high


def compute_transform(img, low_pct=LOW_PCT, high_pct=HIGH_PCT):
    """Calcula (scale[C], shift[C]) que mapea [low, high] -> [0, 255] por canal.

    out = clip((in - shift) * scale, 0, 255). Si un canal es plano (high==low),
    queda como IDENTIDAD (scale=1, shift=0): no se toca ese canal. Restarle su
    propio `low` lo colapsaria a 0 (p. ej. un canal saturado a 255 por
    sobreexposicion se volveria negro), destruyendo informacion en vez de
    preservarla.
    """
    low, high = channel_bounds(img, low_pct, high_pct)
    span = high - low
    flat = span <= EPS
    scale = np.where(flat, 1.0, 255.0 / np.maximum(span, EPS))
    shift = np.where(flat, 0.0, low)
    return scale, shift


def apply_transform(img, scale, shift):
    """Aplica out = clip((img - shift) * scale, 0, 255) y devuelve uint8."""
    arr = np.asarray(img, dtype=np.float64)
    scale = np.asarray(scale, dtype=np.float64)
    shift = np.asarray(shift, dtype=np.float64)
    out = (arr - shift) * scale
    return np.clip(out, 0, 255).astype(np.uint8)


def correct(img, low_pct=LOW_PCT, high_pct=HIGH_PCT):
    """Conveniencia: calcula la transformacion de la propia imagen y la aplica."""
    scale, shift = compute_transform(img, low_pct, high_pct)
    return apply_transform(img, scale, shift)
