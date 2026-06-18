#!/usr/bin/env bash
# Flashea la microSD para un DB21-J4 con la distro ente.
# Uso: tools/flash_sd.sh <hostname> <wifi_ssid> <wifi_pass>
set -euo pipefail
HOST="${1:?Uso: flash_sd.sh <hostname> <wifi_ssid> <wifi_pass>}"
SSID="${2:?falta wifi_ssid}"
PASS="${3:?falta wifi_pass}"

echo "==> ADVERTENCIA: esto BORRA la microSD seleccionada."
echo "    Robot: ${HOST}  |  Config: DB21J  |  Distro: ente"
read -r -p "Continuar? (escribe SI): " ok
[ "$ok" = "SI" ] || { echo "Cancelado."; exit 1; }

dts init_sd_card \
  --hostname "${HOST}" \
  --type duckiebot \
  --configuration DB21J \
  --wifi "${SSID}:${PASS}" \
  --country CL

echo "OK: SD flasheada. Inserta en el robot, enciende y espera ~5 min."
echo "    Luego: tools/check_robot.sh ${HOST}"
