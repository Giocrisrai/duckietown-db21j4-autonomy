"""Selección de dispositivo de cómputo: cuda (Colab) / mps (M1) / cpu."""


def _cuda_ok() -> bool:
    """Verifica si CUDA está disponible."""
    try:
        import torch

        return torch.cuda.is_available()
    except Exception:
        return False


def _mps_ok() -> bool:
    """Verifica si MPS (Metal Performance Shaders, M1/M2) está disponible."""
    try:
        import torch

        return torch.backends.mps.is_available()
    except Exception:
        return False


def select_device(prefer: str = "auto") -> str:
    """Selecciona el dispositivo de cómputo.

    Args:
        prefer: "auto" (selecciona cuda > mps > cpu), "cpu", "cuda" o "mps".

    Returns:
        String con el dispositivo seleccionado.
    """
    if prefer != "auto":
        return prefer
    if _cuda_ok():
        return "cuda"
    if _mps_ok():
        return "mps"
    return "cpu"
