#!/bin/bash
# Auto-arranque al boot: VNC (x11vnc) + camara + no-dormir.
# SEGURIDAD: escucha SOLO en la IP del enlace USB (BIND, no 0.0.0.0); usa la clave
# de ~/.vnc/passwd (creala antes con setup_vnc_nosudo.sh). VNC va en claro -> solo
# para el enlace USB privado; en redes no confiables usa tunel SSH.
BIND="${VNC_BIND:-192.168.55.1}"
for i in $(seq 1 40); do
  A=$(ps -o args= -C Xorg 2>/dev/null | grep -oE "/[^ ]*Xauthority" | head -1)
  [ -n "$A" ] && [ -e "$A" ] && break
  sleep 2
done
export DISPLAY=:0 XAUTHORITY="$A" DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export LD_LIBRARY_PATH=$HOME/vnc/usr/lib/aarch64-linux-gnu
xset s off -dpms s noblank 2>/dev/null
gsettings set org.gnome.desktop.screensaver lock-enabled false 2>/dev/null
gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing 2>/dev/null
pkill x11vnc 2>/dev/null; sleep 1
[ -f "$HOME/.vnc/passwd" ] || { echo "FALTA ~/.vnc/passwd (corre setup_vnc_nosudo.sh)"; exit 1; }
LISTEN_OPT="-listen $BIND"; [ "$BIND" = "localhost" ] && LISTEN_OPT="-localhost"
"$HOME/vnc/usr/bin/x11vnc" -display :0 -auth "$A" -forever -rfbauth "$HOME/.vnc/passwd" \
  $LISTEN_OPT -rfbport 5900 -bg -o /tmp/x11vnc.log
bash "$HOME/camera_viewer.sh" >/tmp/camview.log 2>&1 &
