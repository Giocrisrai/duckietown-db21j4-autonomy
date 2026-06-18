#!/usr/bin/env bash
# Construye el engine TensorRT desde un ONNX. EJECUTAR EN LA JETSON NANO.
# Uso: build_trt_engine.sh model.onnx model.engine
set -euo pipefail
ONNX="${1:?Uso: build_trt_engine.sh <model.onnx> <model.engine>}"
ENGINE="${2:?falta ruta de salida .engine}"

TRTEXEC="/usr/src/tensorrt/bin/trtexec"
[ -x "$TRTEXEC" ] || { echo "ERROR: no se encontro trtexec en $TRTEXEC"; exit 1; }

echo "==> Construyendo engine FP16 (puede tardar varios minutos)..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --workspace=2048

echo "OK: engine guardado en $ENGINE"
