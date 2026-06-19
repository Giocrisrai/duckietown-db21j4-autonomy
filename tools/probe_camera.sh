#!/usr/bin/env bash
# Prueba QUE camara/pipeline funciona en la Jetson (ayuda a integrar IMX219-B y
# a elegir alternativa). Itera sensor-id CSI (nvargus) y dispositivos USB/V4L2.
# Uso (en el robot):  tools/probe_camera.sh
set -uo pipefail

echo "==> Sensores detectados por el kernel (dmesg):"
( dmesg 2>/dev/null | grep -iE "imx219|imx477|camera|tegra-camera" | tail -6 ) || echo "  (sin acceso a dmesg)"
command -v v4l2-ctl >/dev/null 2>&1 && { echo "==> v4l2 devices:"; v4l2-ctl --list-devices 2>/dev/null; }

echo "==> Probando CSI (nvargus) sensor-id 0 y 1:"
for sid in 0 1; do
  out="/tmp/cam_csi_${sid}.jpg"
  if timeout 12 gst-launch-1.0 -q nvarguscamerasrc sensor-id="$sid" num-buffers=1 ! \
       "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1" ! nvjpegenc ! \
       filesink location="$out" >/dev/null 2>&1 && [ -s "$out" ]; then
    echo "  [OK] CSI sensor-id=${sid} captura ($out)"
  else
    echo "  [no] CSI sensor-id=${sid}"
  fi
done

echo "==> Probando camaras USB/V4L2:"
for dev in /dev/video0 /dev/video1 /dev/video2; do
  [ -e "$dev" ] || continue
  out="/tmp/cam_usb_$(basename "$dev").jpg"
  if timeout 10 gst-launch-1.0 -q v4l2src device="$dev" num-buffers=1 ! jpegenc ! \
       filesink location="$out" >/dev/null 2>&1 && [ -s "$out" ]; then
    echo "  [OK] USB ${dev} captura ($out)"
  else
    echo "  [no] USB ${dev}"
  fi
done

echo ""
echo "Si NINGUNA CSI funciona (caso IMX219-B): habilita el overlay del sensor con"
echo "  sudo /opt/nvidia/jetson-io/jetson-io.py   -> Configure CSI -> Camera IMX219 (y reinicia)."
echo "Como alternativa, una camara USB/UVC funciona sin overlay (modo 'usb' del stack)."
