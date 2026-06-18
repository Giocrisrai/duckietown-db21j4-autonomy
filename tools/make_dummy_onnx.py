#!/usr/bin/env python3
"""CLI: genera un modelo ONNX sintetico de prueba (envuelve sim_perception.make_dummy_onnx)."""
import sys
from pathlib import Path
import numpy as np

# Permite ejecutar desde la raiz del repo sin instalar el paquete
_RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_RAIZ / "simulation"))

from sim_perception import make_dummy_onnx

def main():
    salida = sys.argv[1] if len(sys.argv) > 1 else "dummy.onnx"
    dets = np.array([[[100., 100., 200., 200., 0.9, 0.]]], dtype=np.float32)
    ruta = make_dummy_onnx(salida, dets)
    print(f"ONNX sintetico escrito en: {ruta}")

if __name__ == "__main__":
    main()
