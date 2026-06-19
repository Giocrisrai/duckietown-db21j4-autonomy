#!/usr/bin/env bash
# Despliega al Duckiebot SIN dts: lleva la imagen y levanta los contenedores
# con docker compose directamente en el robot.
# Uso:
#   tools/deploy_robot.sh registry <hostname> <registry/img>
#   tools/deploy_robot.sh tar <hostname> <imagen.tar>
set -euo pipefail
MODE="${1:?registry|tar}"; HOST="${2:?hostname}"
SSHH="duckie@${HOST}.local"
case "$MODE" in
  registry) IMG="${3:?falta registry/img}"; echo "==> pull en el robot"; ssh "$SSHH" "docker pull '$IMG'" ;;
  tar) TAR="${3:?falta imagen.tar}"; echo "==> scp + docker load"; scp "$TAR" "$SSHH:/tmp/img.tar"; ssh "$SSHH" "docker load -i /tmp/img.tar" ;;
  *) echo "modo invalido (registry|tar)"; exit 1 ;;
esac
echo "==> Copiando compose y levantando el stack en ${HOST}"
scp bringup/docker-compose.yaml "$SSHH:/tmp/dt-compose.yaml"
ssh "$SSHH" "VEHICLE_NAME='${HOST}' docker compose -f /tmp/dt-compose.yaml up -d"
echo "OK: contenedores levantados en ${HOST} (sin dts). Logs: ssh ${SSHH} 'docker compose -f /tmp/dt-compose.yaml logs -f'"
