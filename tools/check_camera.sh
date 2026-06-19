#!/usr/bin/env bash
# Diagnostico de COMPATIBILIDAD de la camara del Duckiebot (Jetson Nano CSI).
# Detecta y prueba la camara a nivel hardware (driver/CSI/nvargus) y ROS.
# Uso (en el robot, o vía ssh):  tools/check_camera.sh [hostname]
#   - sin hostname: corre los chequeos de hardware locales (en la Nana).
#   - con hostname: corre los chequeos de ROS contra ese Duckiebot.
set -uo pipefail
ROBOT="${1:-}"

echo "==> 1) Dispositivos de video (V4L2)"
ls -l /dev/video* 2>/dev/null && echo "  [OK] hay /dev/video*" || echo "  [WARN] no se ven /dev/video* (¿cable CSI suelto? ¿driver?)"

echo "==> 2) Sensor CSI detectado por el kernel"
( dmesg 2>/dev/null | grep -iE "imx219|csi|camera|tegra-camera" | tail -5 ) || echo "  [WARN] sin acceso a dmesg"
[ -e /proc/device-tree/ ] && (find /proc/device-tree -iname "*imx219*" 2>/dev/null | head -1 && echo "  [OK] IMX219 en device-tree" || echo "  [INFO] IMX219 no listado en device-tree")

echo "==> 3) v4l2-ctl (formatos soportados)"
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices 2>/dev/null
  v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | head -20
else
  echo "  [INFO] v4l2-ctl no instalado (apt install v4l-utils)"
fi

echo "==> 4) nvargus / GStreamer CSI (prueba de captura de 1 frame)"
if command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "  Probando pipeline nvarguscamerasrc -> 1 frame a /tmp/cam_test.jpg ..."
  timeout 15 gst-launch-1.0 -q nvarguscamerasrc num-buffers=1 ! \
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1" ! \
    nvjpegenc ! filesink location=/tmp/cam_test.jpg 2>/dev/null \
    && [ -s /tmp/cam_test.jpg ] && echo "  [OK] captura CSI exitosa ($(wc -c </tmp/cam_test.jpg) bytes)" \
    || echo "  [FALLA] nvarguscamerasrc no capturó (revisa nvargus-daemon: sudo systemctl status nvargus-daemon)"
else
  echo "  [INFO] gst-launch-1.0 no disponible aquí"
fi

if [ -n "$ROBOT" ]; then
  echo "==> 5) Tópico ROS de la cámara en ${ROBOT}"
  echo "  Frecuencia (~30 Hz esperado):"
  timeout 8 rostopic hz "/${ROBOT}/camera_node/image/compressed" 2>/dev/null || echo "  [WARN] sin datos en el tópico (¿camera_node arrancó?)"
  echo "  Resolución (camera_info):"
  timeout 6 rostopic echo -n1 "/${ROBOT}/camera_node/camera_info" 2>/dev/null | grep -E "^width:|^height:" || echo "  [WARN] sin camera_info"
else
  echo "==> 5) (omitido) Para chequear ROS: tools/check_camera.sh <hostname>"
fi

echo "FIN. Resumen rápido de compatibilidad arriba ([OK]/[WARN]/[FALLA])."
