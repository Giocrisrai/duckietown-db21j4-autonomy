#!/usr/bin/env bash
# Instala/actualiza la Duckietown Shell (dts) y verifica la version.
set -euo pipefail

echo "==> Instalando duckietown-shell..."
pip3 install --user --upgrade duckietown-shell

echo "==> Version instalada:"
dts version || { echo "ERROR: dts no quedo en PATH. Agrega ~/.local/bin al PATH"; exit 1; }

echo "==> Configurando distro ente..."
dts --set-version ente 2>/dev/null || true

echo "OK: dts listo. Si es la primera vez, ejecuta: dts tok set"
