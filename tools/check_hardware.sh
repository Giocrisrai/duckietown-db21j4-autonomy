#!/usr/bin/env bash
# Diagnostico de FALLAS de hardware del Duckiebot via topicos ROS.
# Uso (en el robot o vía ssh):  tools/check_hardware.sh <hostname>
set -uo pipefail
ROBOT="${1:?Uso: check_hardware.sh <hostname>}"
NS="/${ROBOT}"

chk() { # nombre topico timeout
  printf "  %-12s " "$1:"
  timeout "${3:-5}" rostopic echo -n1 "$2" >/dev/null 2>&1 \
    && echo "[OK] publica $2" || echo "[FALLA] sin datos en $2"
}

echo "==> Bateria"
timeout 5 rostopic echo -n1 "${NS}/battery/info" 2>/dev/null | grep -E "percentage|voltage" || echo "  [WARN] sin ${NS}/battery/info"
echo "==> Motores / ruedas (comando)"; chk "wheels" "${NS}/wheels_driver_node/wheels_cmd"
echo "==> Encoders"
chk "enc_izq" "${NS}/left_wheel_encoder_node/tick"
chk "enc_der" "${NS}/right_wheel_encoder_node/tick"
echo "==> IMU"; chk "imu" "${NS}/imu_node/data"
echo "==> ToF"; chk "tof" "${NS}/front_center_tof_driver_node/range"
echo "==> Camara"; chk "camara" "${NS}/camera_node/image/compressed"
echo "FIN. Revisa [OK]/[FALLA] por componente."
