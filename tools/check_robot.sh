#!/usr/bin/env bash
# Diagnostico del Duckiebot: conectividad, SSH, contenedores y camara.
# Uso: tools/check_robot.sh <hostname>   (ej: tools/check_robot.sh quackbot)
set -uo pipefail
ROBOT="${1:?Uso: check_robot.sh <hostname>}"

echo "==> 1) Ping mDNS (${ROBOT}.local)"
ping -c 2 "${ROBOT}.local" && echo "  [OK] responde" || echo "  [FALLA] sin respuesta mDNS"

echo "==> 2) Estado general dts"
dts fleet discover || echo "  [WARN] no se pudo listar la flota"

echo "==> 3) Contenedores en el robot"
dts duckiebot diagnose "${ROBOT}" 2>/dev/null || \
  echo "  [WARN] diagnose no disponible; revisa el dashboard http://${ROBOT}.local"

echo "==> 4) Dashboard"
echo "  Abre: http://${ROBOT}.local/  (camara, motores, bateria)"
echo "FIN del diagnostico."
