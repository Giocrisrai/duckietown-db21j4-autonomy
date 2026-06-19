#!/usr/bin/env bash
# Construye la imagen del robot (arm64) SIN dts, usando docker buildx.
# Funciona desde cualquier host (x86 o Mac arm) via emulacion.
# Uso:
#   tools/build_image.sh                              # build local (--load)
#   tools/build_image.sh <tag> --push <registry/img> # publica a un registry
#   tools/build_image.sh <tag> --save <imagen.tar>   # guarda un tarball portatil
set -euo pipefail
TAG="${1:-duckietown-new-version:latest}"
PLATFORM="${PLATFORM:-linux/arm64}"
echo "==> buildx ${PLATFORM} -> ${TAG} (sin dts)"
command -v docker >/dev/null 2>&1 || { echo "ERROR: falta docker"; exit 1; }
docker buildx inspect dtbuilder >/dev/null 2>&1 || docker buildx create --name dtbuilder --use
OUT="--load"
case "${2:-}" in
  --push) TAG="${3:?falta registry/img}"; OUT="--push" ;;
  --save) OUT=(-o "type=docker,dest=${3:?falta imagen.tar}") ;;
esac
docker buildx build --platform "$PLATFORM" -t "$TAG" "${OUT[@]:-$OUT}" .
echo "OK: imagen ${TAG} construida para ${PLATFORM}"
