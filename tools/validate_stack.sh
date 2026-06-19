#!/usr/bin/env bash
# Validacion INTEGRAL y multiplataforma del stack (sin robot).
# Corre tests y valida configs/launches/scripts/compose. Uso: tools/validate_stack.sh
set -uo pipefail
fail=0
PY="${PYTHON:-python3}"

echo "==> 1) Suite de tests"
$PY -m pytest -q || fail=1

echo "==> 2) Launches XML bien formados"
while IFS= read -r f; do
  $PY -c "import xml.dom.minidom; xml.dom.minidom.parse('$f')" 2>/dev/null \
    && echo "  [OK] $f" || { echo "  [FALLA] $f"; fail=1; }
done < <(find . -name "*.launch" -not -path "*/.venv/*")

echo "==> 3) YAML (compose, CI) y .dtproject presente"
for f in bringup/docker-compose.yaml docker-compose.dev.yaml .github/workflows/tests.yml; do
  [ -f "$f" ] && { $PY -c "import yaml; yaml.safe_load(open('$f'))" 2>/dev/null \
    && echo "  [OK] $f" || { echo "  [FALLA] $f"; fail=1; }; }
done
[ -f .dtproject ] && echo "  [OK] .dtproject" || { echo "  [FALLA] falta .dtproject"; fail=1; }

echo "==> 4) Sintaxis de scripts bash"
for s in tools/*.sh; do
  bash -n "$s" && echo "  [OK] $s" || { echo "  [FALLA] $s"; fail=1; }
done

echo "==> 5) Docker / contenedores (si hay Docker en el host)"
if command -v docker >/dev/null 2>&1; then
  docker compose -f bringup/docker-compose.yaml config >/dev/null 2>&1 \
    && echo "  [OK] bringup/docker-compose.yaml valido" || echo "  [WARN] revisar bringup compose"
  docker compose -f docker-compose.dev.yaml config >/dev/null 2>&1 \
    && echo "  [OK] docker-compose.dev.yaml valido" || echo "  [WARN] revisar dev compose"
else
  echo "  [INFO] Docker no disponible aqui; el bring-up se valida en el host con Docker."
fi

if [ "$fail" = 0 ]; then echo "VALIDACION OK"; else echo "VALIDACION CON FALLOS"; exit 1; fi
