#!/bin/bash
# Instala y arranca x11vnc en un Jetson SIN sudo: baja el .deb + deps a ~/vnc y
# corre el binario como el usuario, atado al display :0 (que el usuario ya puede
# leer en una sesion grafica con auto-login). Pensado para acceso por USB privado.
set -e
cd ~ && rm -rf ~/vnc && mkdir -p ~/vnc && cd ~/vnc
apt-get download x11vnc libvncserver1 libavahi-client3 libavahi-common3
for d in *.deb; do dpkg-deb -x "$d" .; done
mkdir -p ~/.vnc
PASS="${1:-duckie}"
LD_LIBRARY_PATH=$HOME/vnc/usr/lib/aarch64-linux-gnu ~/vnc/usr/bin/x11vnc -storepasswd "$PASS" ~/.vnc/passwd
A=$(ps -o args= -C Xorg | grep -oE "/[^ ]*Xauthority" | head -1)
LD_LIBRARY_PATH=$HOME/vnc/usr/lib/aarch64-linux-gnu ~/vnc/usr/bin/x11vnc \
  -display :0 -auth "$A" -forever -rfbauth ~/.vnc/passwd -rfbport 5900 -bg -o /tmp/x11vnc.log
echo "VNC en :5900 (clave: $PASS). Conecta: vnc://<ip-robot>:5900"
