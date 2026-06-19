#!/usr/bin/env bash
# Lista los discos para identificar la microSD ANTES de flashear (seguridad).
# macOS: usa diskutil; Linux: usa lsblk.
set -uo pipefail
case "$(uname -s)" in
  Darwin)
    echo "== Discos (macOS) — busca tu SD (Apple_... externo, tamano de la SD) =="
    diskutil list
    echo ""
    echo "La SD suele ser /dev/diskN externo. Para dd usa el RAW: /dev/rdiskN (mas rapido)."
    echo "Desmonta antes:  diskutil unmountDisk /dev/diskN"
    ;;
  Linux)
    echo "== Discos (Linux) — busca tu SD por tamano =="
    lsblk -o NAME,SIZE,TYPE,MOUNTPOINT,MODEL
    echo ""
    echo "La SD suele ser /dev/sdX o /dev/mmcblkN. Desmonta sus particiones antes de flashear."
    ;;
  *) echo "SO no reconocido: $(uname -s)";;
esac
