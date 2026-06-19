#!/usr/bin/env bash
# Bootstrap MULTIPLATAFORMA del entorno de desarrollo/entrenamiento.
# Funciona en macOS y en cualquier Linux. Crea un venv, instala deps y valida.
# Uso:  tools/setup_env.sh
set -euo pipefail
OS="$(uname -s)"; ARCH="$(uname -m)"
echo "==> Plataforma detectada: ${OS} / ${ARCH}"
PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null 2>&1 || { echo "ERROR: falta python3 (instala Python 3.9+)"; exit 1; }
echo "==> Creando entorno virtual .venv"
"$PY" -m venv .venv
# shellcheck disable=SC1091
. .venv/bin/activate
pip install -q -U pip
echo "==> Instalando dependencias de desarrollo (requirements-dev.txt)"
pip install -q -r requirements-dev.txt
echo "==> Validando con la suite de tests"
python -m pytest -q
echo "OK: entorno listo en .venv para ${OS}/${ARCH}."
echo "    Para Deep RL:  pip install -r requirements-rl.txt"
echo "    Para entrenar YOLO:  pip install -r training/requirements.txt"
