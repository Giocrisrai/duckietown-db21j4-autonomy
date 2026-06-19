#!/usr/bin/env bash
# Flashea una imagen base a la microSD SIN dts (balena CLI o dd). Multiplataforma.
# Uso: tools/flash_sd_raw.sh <imagen.img|.img.xz> <dispositivo>
#   macOS:  dispositivo /dev/rdiskN     (ver: diskutil list)
#   Linux:  dispositivo /dev/sdX|mmcblkN (ver: lsblk)
set -euo pipefail
IMG="${1:?ruta a la imagen (.img/.img.xz)}"; DEV="${2:?dispositivo de la SD}"
[ -f "$IMG" ] || { echo "ERROR: no existe $IMG"; exit 1; }
echo "ADVERTENCIA: se BORRARA ${DEV} por completo."
read -r -p "Escribe SI para continuar: " ok; [ "$ok" = "SI" ] || { echo "Cancelado"; exit 1; }
if command -v balena >/dev/null 2>&1; then
  balena local flash "$IMG" --drive "$DEV" --yes
else
  echo "==> balena CLI no disponible; usando dd"
  if [ "$(uname -s)" = "Darwin" ]; then BS="4m"; else BS="4M"; fi
  case "$IMG" in
    *.xz) xzcat "$IMG" | sudo dd of="$DEV" bs="$BS" ;;
    *)    sudo dd if="$IMG" of="$DEV" bs="$BS" ;;
  esac
  sync
fi
echo "OK: SD flasheada con ${IMG} (sin dts)."
