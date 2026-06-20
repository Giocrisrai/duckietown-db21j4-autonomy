#!/usr/bin/env bash
# Validacion COMPLETA del Duckiebot DB21-J4 en UN comando (robot-side).
#
# Corre, en orden y con compuerta de seguridad por el bus I2C 1:
#   0) Preflight   - bus 1 responde? escaneo de PCA9685/IMU/ToF/OLED
#   1) Sensores    - IMU WHO_AM_I, ToF, OLED (read-only)
#   2) Camara      - /dev/video*, dmesg IMX/OV, overlay
#   3) Calibracion - valida/genera kinematics + camara
#   4) Motores     - secuencia segura por el driver OFICIAL (solo con --motors,
#                    y SOLO si el bus 1 responde y el PCA9685 esta presente)
#
# Uso (en el robot):
#   sudo bash tools/validate_robot.sh            # todo menos mover motores
#   sudo bash tools/validate_robot.sh --motors   # incluye girar motores (ruedas en el aire!)
#
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BUS=1
IMG="duckietown/dt-duckiebot-interface:daffy-arm64v8"
DO_MOTORS=0
[ "${1:-}" = "--motors" ] && DO_MOTORS=1

ok()   { printf "  \033[32m[OK]\033[0m   %s\n" "$1"; }
bad()  { printf "  \033[31m[FALLA]\033[0m %s\n" "$1"; }
warn() { printf "  \033[33m[AVISO]\033[0m %s\n" "$1"; }
hdr()  { printf "\n== %s ==\n" "$1"; }

PCA_OK=0; BUS1_OK=0

# ---------- 0) Preflight ----------
hdr "0) Preflight - bus I2C $BUS"
if timeout 6 i2cdetect -y -r "$BUS" >/tmp/_i2c1.txt 2>/dev/null; then
  BUS1_OK=1; ok "bus $BUS responde (no esta colgado)"
  scan="$(cat /tmp/_i2c1.txt)"
  for a in 40 60; do echo "$scan" | grep -qiE " $a " && { PCA_OK=1; ok "PCA9685 (motores) 0x$a presente"; }; done
  [ "$PCA_OK" = 0 ] && bad "PCA9685 (0x40/0x60) NO presente"
  echo "$scan" | grep -qiE " 68 " && ok "IMU 0x68 presente"   || warn "IMU 0x68 ausente"
  echo "$scan" | grep -qiE " 29 " && ok "ToF 0x29 presente"   || warn "ToF 0x29 ausente"
  echo "$scan" | grep -qiE " 3c " && ok "OLED 0x3c presente"  || warn "OLED 0x3c ausente"
else
  bad "bus $BUS COLGADO (timeout) -> fallo fisico del header 40-pin / HUT."
  warn "Se omiten sensores y motores (no se puede comunicar por I2C)."
fi

# ---------- 1) Sensores ----------
hdr "1) Sensores (IMU/ToF/OLED)"
if [ "$BUS1_OK" = 1 ]; then
  python3 "$REPO/tools/test_sensors.py" "$BUS" || warn "test_sensors reporto faltantes"
else
  warn "omitido (bus $BUS colgado)"
fi

# ---------- 2) Camara ----------
hdr "2) Camara"
if ls /dev/video* >/dev/null 2>&1; then
  ok "dispositivo de video presente: $(ls /dev/video* | tr '\n' ' ')"
else
  bad "/dev/video* ausente"
  cam="$(dmesg 2>/dev/null | grep -iE 'imx|ov5647|camera' | tail -3)"
  [ -n "$cam" ] && echo "$cam" | sed 's/^/    /'
  echo "$cam" | grep -q -- '-121' && warn "error -121 = flex CSI mal contactado (reasentar flex)"
fi

# ---------- 3) Calibracion ----------
hdr "3) Calibracion"
KIN="$REPO/config/calibrations/kinematics/prueba.yaml"
CAM="$REPO/config/calibrations/camera_intrinsic/prueba.yaml"
[ -f "$KIN" ] || python3 "$REPO/tools/calibrate.py" generate --robot prueba --camera pi-m12 >/dev/null
python3 "$REPO/tools/calibrate.py" validate "$KIN" && ok "kinematics valida"   || bad "kinematics invalida"
python3 "$REPO/tools/calibrate.py" validate "$CAM" && ok "camara valida"        || bad "camara invalida"

# ---------- 4) Motores ----------
hdr "4) Motores"
if [ "$DO_MOTORS" = 0 ]; then
  warn "omitido (pasa --motors para girarlos; ruedas en el aire!)"
elif [ "$BUS1_OK" = 0 ] || [ "$PCA_OK" = 0 ]; then
  bad "no se ejecuta: bus colgado o PCA9685 ausente (compuerta de seguridad)"
else
  echo "  Girando motores con el driver oficial (secuencia segura, PWM <=40%)..."
  PYP="/code/catkin_ws/src/dt-duckiebot-interface/packages/hat_driver/include:/work/control"
  docker run --rm --privileged -v /dev:/dev -v "$REPO":/work \
    --entrypoint bash "$IMG" -lc \
    "export PYTHONPATH=$PYP; python3 /work/tools/robot_motor_test.py" \
    && ok "secuencia de motores completada" || bad "fallo al comandar motores"
fi

hdr "Resumen"
echo "  Jetson/CPU/red: ver salud con 'tools/check_hardware.sh'. Bus1=$([ $BUS1_OK = 1 ] && echo OK || echo COLGADO), PCA9685=$([ $PCA_OK = 1 ] && echo OK || echo NO)."
echo "  Cuando bus1=OK y PCA9685=OK, repite con --motors para girar las ruedas."
