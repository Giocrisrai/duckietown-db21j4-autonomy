"""Letterbox: redimensiona manteniendo aspecto y rellena hasta new_shape."""


def compute_letterbox(h: int, w: int, new_shape):
    """
    Calcula parámetros para letterbox sin tocar pixeles (testeable).

    Args:
        h: altura original de la imagen.
        w: ancho original de la imagen.
        new_shape: tamaño objetivo. Si es int, se usa cuadrado (H, W).

    Returns:
        Tupla (r, (new_w, new_h), (dw, dh)):
            - r: factor de escala min(new_shape[0]/h, new_shape[1]/w)
            - (new_w, new_h): dimensiones escaladas (manteniendo aspecto)
            - (dw, dh): padding izquierdo/superior para centrar
    """
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)  # (H, W)

    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))  # (w, h)
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    return r, new_unpad, (dw, dh)


def letterbox(img, new_shape=416, color=(114, 114, 114)):
    """
    Aplica letterbox a una imagen BGR (numpy).

    Args:
        img: imagen numpy (H, W, C) en formato BGR.
        new_shape: tamaño objetivo (default 416).
        color: color de padding en BGR (default gris neutro).

    Returns:
        Tupla (img_out, r, (left, top)):
            - img_out: imagen con letterbox aplicado
            - r: factor de escala usado
            - (left, top): coordenadas del padding izquierdo/superior
    """
    import cv2

    h, w = img.shape[:2]
    r, new_unpad, (dw, dh) = compute_letterbox(h, w, new_shape)
    resized = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    out = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return out, r, (left, top)
