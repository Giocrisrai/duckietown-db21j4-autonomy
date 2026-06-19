#!/usr/bin/env bash
# Bootstrap MULTIPLATAFORMA del entorno de desarrollo/entrenamiento.
# Funciona en macOS y en cualquier Linux. Crea un venv, instala deps y valida.
# Uso:  tools/setup_env.sh
set -euo pipefail
OS="$(uname -s)"; ARCH="$(uname -m)"
echo "==> Plataforma detectada: ${OS} / ${ARCH}"
if command -v uv >/dev/null 2>&1; then
  # uv: gestor moderno (estado del arte), ~10-100x mas rapido que pip.
  echo "==> Usando uv ($(uv --version))"
  uv venv .venv
  # shellcheck disable=SC1091
  . .venv/bin/activate
  uv pip install -r requirements-dev.txt
  INST="uv pip install"
else
  PY="${PYTHON:-python3}"
  command -v "$PY" >/dev/null 2>&1 || { echo "ERROR: falta python3 (instala Python 3.9+) o uv"; exit 1; }
  echo "==> uv no encontrado; usando venv+pip (instala uv para mayor velocidad: https://docs.astral.sh/uv/)"
  "$PY" -m venv .venv
  # shellcheck disable=SC1091
  . .venv/bin/activate
  pip install -q -U pip
  pip install -q -r requirements-dev.txt
  INST="pip install"
fi
echo "==> Validando con la suite de tests"
python -m pytest -q
echo "OK: entorno listo en .venv para ${OS}/${ARCH}."
echo "    Para Deep RL:        ${INST} -r requirements-rl.txt"
echo "    Para entrenar YOLO:  ${INST} -r training/requirements.txt"
