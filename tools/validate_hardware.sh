#!/usr/bin/env bash
# Validacion END-TO-END de hardware (preflight) antes de operar/flashear.
# Agrega todos los chequeos del robot. Uso: tools/validate_hardware.sh <hostname>
set -uo pipefail
ROBOT="${1:?Uso: validate_hardware.sh <hostname>}"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "================ PREFLIGHT DE HARDWARE: ${ROBOT} ================"
echo "### 1/4 Conectividad y contenedores ###";  "$HERE/check_robot.sh"    "$ROBOT" || true
echo "### 2/4 Camara ###";                       "$HERE/check_camera.sh"   "$ROBOT" || true
echo "### 3/4 Componentes (motores/encoders/IMU/ToF/bateria) ###"; "$HERE/check_hardware.sh" "$ROBOT" || true
echo "### 4/4 Red ###";                          "$HERE/setup_network.sh"  check "$ROBOT" || true
echo "================================================================"
echo "Revisa [OK]/[WARN]/[FALLA] por componente."
echo "Monitoreo EN VIVO mientras corre:  python -m run_monitor  (en el robot)"
echo "  y conectate por red:  curl http://${ROBOT}.local:8080/"
