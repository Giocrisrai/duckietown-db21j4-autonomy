#!/usr/bin/env bash
# Configuracion y diagnostico de RED del Duckiebot.
# Uso:
#   tools/setup_network.sh check <hostname>           # diagnostica conectividad
#   tools/setup_network.sh wifi <hostname> <ssid> <pass>   # (re)configura WiFi via dts
set -uo pipefail
CMD="${1:?Uso: setup_network.sh check|wifi ...}"

case "$CMD" in
  check)
    ROBOT="${2:?falta hostname}"
    echo "==> Ping mDNS (${ROBOT}.local)"; ping -c 2 "${ROBOT}.local" && echo "  [OK]" || echo "  [FALLA] sin mDNS"
    echo "==> SSH"; timeout 6 ssh -o ConnectTimeout=4 -o BatchMode=yes "duckie@${ROBOT}.local" "hostname" 2>/dev/null && echo "  [OK] ssh" || echo "  [WARN] ssh no disponible (clave/credenciales)"
    echo "==> Dashboard"; echo "  http://${ROBOT}.local/"
    ;;
  wifi)
    ROBOT="${2:?falta hostname}"; SSID="${3:?falta ssid}"; PASS="${4:?falta pass}"
    echo "==> Reconfigurando WiFi de ${ROBOT} a SSID=${SSID} (via dts)"
    echo "    NOTA: el cambio de WiFi en Duckietown se hace re-flasheando con --wifi"
    echo "    o editando /var/local/wifi en el robot. Comando recomendado:"
    echo "    dts duckiebot wifi ${ROBOT} 2>/dev/null || echo 'usa tools/flash_sd.sh con --wifi \"${SSID}:${PASS}\"'"
    ;;
  *) echo "Comando desconocido: $CMD"; exit 1 ;;
esac
