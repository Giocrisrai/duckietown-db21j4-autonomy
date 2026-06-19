#!/usr/bin/env bash
# Detecta el microcontrolador SERIAL de la HUT del Duckiebot DB21-J4.
# En el DB21-J4 los motores/encoders/LEDs van por /dev/ttyACM* (Atmel 04d8:ecb7),
# NO por I2C. Uso (en el robot): tools/check_hut.sh
set -uo pipefail
echo "== HUT (microcontrolador serial) del DB21-J4 =="
found=""
for d in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$d" ] || continue
  vid=$(udevadm info -q property -n "$d" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')
  pid=$(udevadm info -q property -n "$d" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p')
  if [ "$vid" = "04d8" ] && [ "$pid" = "ecb7" ]; then
    echo "  [OK] HUT en $d  (Atmel ${vid}:${pid}, 'Virtual COM-Port')"; found="$d"
  else
    echo "  $d -> ${vid:-?}:${pid:-?} (otro dispositivo)"
  fi
done
if [ -n "$found" ]; then
  echo "RESULTADO: HUT viva en $found -> motores/encoders/LEDs disponibles por serial."
  echo "  Para comandarlos: dt-duckiebot-interface (protocolo HUT) sobre $found."
else
  echo "RESULTADO: HUT serial NO detectada. Revisa el cable USB de la HUT y la alimentacion."
fi
