"""Tests de anti-Instagram (normalizacion de color/iluminacion)."""
import numpy as np
from object_detection.anti_instagram import (
    channel_bounds, compute_transform, apply_transform, correct,
)


def _img(valores):
    """Imagen pequena BGR a partir de una lista de valores por pixel."""
    return np.array(valores, dtype=np.uint8).reshape(-1, 1, 3)


def test_channel_bounds_por_canal():
    # canal B en 10..20, G en 100..110, R en 200..210
    img = np.stack([
        np.linspace(10, 20, 50),
        np.linspace(100, 110, 50),
        np.linspace(200, 210, 50),
    ], axis=-1).reshape(50, 1, 3).astype(np.uint8)
    low, high = channel_bounds(img, low_pct=0, high_pct=100)
    assert low[0] < 15 < high[0]
    assert low[2] > 190                       # canal rojo arriba
    assert (high > low).all()


def test_imagen_oscura_se_estira_a_rango_completo():
    # imagen sub-expuesta (valores 0..50) -> tras corregir cubre casi todo [0,255]
    oscura = np.linspace(0, 50, 300).reshape(100, 1, 3).astype(np.uint8)
    out = correct(oscura, low_pct=0, high_pct=100)
    assert out.max() >= 250
    assert out.min() <= 5


def test_apply_transform_clip_y_uint8():
    img = _img([[0, 0, 0], [255, 255, 255]])
    out = apply_transform(img, scale=[2.0, 2.0, 2.0], shift=[0.0, 0.0, 0.0])
    assert out.dtype == np.uint8
    assert out.max() <= 255 and out.min() >= 0          # 255*2 -> clip a 255


def test_canal_plano_no_revienta():
    # un canal constante (span 0) -> scale=1.0, no NaN/inf
    img = np.full((20, 1, 3), [128, 0, 0], dtype=np.uint8)   # G y R planos en 0
    scale, shift = compute_transform(img)
    assert np.isfinite(scale).all()
    out = correct(img)
    assert out.shape == img.shape


def test_canal_plano_no_cero_pasa_identico():
    # Un canal plano en un valor != 0 (p. ej. R saturado a 255 por sobreexposicion)
    # debe pasar SIN CAMBIOS (identidad), no colapsar a 0 al restar su propio shift.
    img = np.full((10, 1, 3), [50, 80, 255], dtype=np.uint8)  # R plano en 255
    out = correct(img)
    assert (out[:, :, 2] == 255).all()                        # R intacto, no a 0


def test_corrige_dominante_de_color():
    # imagen con tinte azul (B alto, R bajo) -> tras corregir, los canales quedan
    # mas balanceados (el rango de cada canal se normaliza a [0,255])
    h = 100
    b = np.linspace(120, 200, h)
    g = np.linspace(40, 90, h)
    r = np.linspace(10, 60, h)
    img = np.stack([b, g, r], axis=-1).reshape(h, 1, 3).astype(np.uint8)
    out = correct(img, low_pct=0, high_pct=100)
    # cada canal ahora abarca ~[0,255]
    for c in range(3):
        assert out[:, :, c].min() <= 5 and out[:, :, c].max() >= 250


def test_transform_reutilizable_es_consistente():
    img = np.linspace(0, 120, 300).reshape(100, 1, 3).astype(np.uint8)
    scale, shift = compute_transform(img)
    a = apply_transform(img, scale, shift)
    b = correct(img)
    assert np.array_equal(a, b)


def test_imagen_vacia_lanza_valueerror():
    import pytest
    with pytest.raises(ValueError):
        channel_bounds(np.zeros((0, 0, 3), dtype=np.uint8))
