#!/bin/bash
# Instala y arranca x11vnc en un Jetson SIN sudo: baja el .deb + deps a ~/vnc y
# corre el binario como el usuario, atado al display :0.
#
# SEGURIDAD:
#   - Requiere una CLAVE como argumento (sin default debil).
#   - Escucha SOLO en la IP del enlace USB device-mode (BIND, por defecto
#     192.168.55.1), NO en 0.0.0.0 -> no se expone en otras interfaces.
#   - VNC va en CLARO. Pensado para el enlace USB privado robot<->host. Para una
#     red no confiable, NO lo expongas: usa un tunel SSH y -localhost:
#       en el host:  ssh -L 5900:localhost:5900 prueba@<ip>
#       y arranca x11vnc con BIND=localhost.
set -e
PASS="${1:?Uso: setup_vnc_nosudo.sh <CLAVE-FUERTE> [BIND-IP]  (sin clave debil por defecto)}"
BIND="${2:-192.168.55.1}"
cd ~ && rm -rf ~/vnc && mkdir -p ~/vnc && cd ~/vnc
apt-get download x11vnc libvncserver1 libavahi-client3 libavahi-common3
for d in *.deb; do dpkg-deb -x "$d" .; done
mkdir -p ~/.vnc
export LD_LIBRARY_PATH=$HOME/vnc/usr/lib/aarch64-linux-gnu
~/vnc/usr/bin/x11vnc -storepasswd "$PASS" ~/.vnc/passwd
chmod 600 ~/.vnc/passwd
A=$(ps -o args= -C Xorg | grep -oE "/[^ ]*Xauthority" | head -1)
LISTEN_OPT="-listen $BIND"; [ "$BIND" = "localhost" ] && LISTEN_OPT="-localhost"
~/vnc/usr/bin/x11vnc -display :0 -auth "$A" -forever -rfbauth ~/.vnc/passwd \
  $LISTEN_OPT -rfbport 5900 -bg -o /tmp/x11vnc.log
echo "VNC en ${BIND}:5900 (clave protegida). Conecta: vnc://${BIND}:5900"
