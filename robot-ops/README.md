# robot-ops — operación remota del Duckiebot (Jetson) por VNC

Herramientas robot-side para **ver, controlar y enfocar** el dispositivo en vivo,
**sin depender de `dts`** y (lo de VNC) **sin sudo**.

| Script | Qué hace |
|---|---|
| `setup_vnc_nosudo.sh [clave]` | Instala x11vnc **sin sudo** (baja .deb + deps a `~/vnc`) y lo arranca atado a `:0`. Conectas con `vnc://<ip>:5900`. |
| `vnc_autostart.sh` | Lo que va en `crontab @reboot`: levanta VNC + cámara + no-dormir en cada boot. |
| `camera_viewer.sh` | Visor de cámara en vivo en el escritorio (gstreamer). |
| `focus_assist.py` | Cámara + **medidor de nitidez** (varianza Laplaciano) para enfocar el lente IMX219 girándolo hasta maximizar el número. |

## Acceso por USB (device-mode): el robot queda en `192.168.55.1`
```bash
# en el robot (una vez):
bash robot-ops/setup_vnc_nosudo.sh 'TU_CLAVE'
crontab -l 2>/dev/null | grep -v vnc_autostart; \
  echo "@reboot /bin/bash $HOME/vnc_autostart.sh" | crontab -   # auto-arranque
# desde el Mac:  Finder -> Cmd+K -> vnc://192.168.55.1:5900
```

## No-dormir / no-bloquear (sin sudo, en la sesión del usuario)
```bash
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing
DISPLAY=:0 xset s off -dpms s noblank
```

## Resolución headless
El driver Tegra de L4T no acepta forzar resolución sin pantalla. Para resolución
real (1080p): **dummy HDMI (~US$5)** o un monitor un momento. Mientras, usar el
zoom del cliente VNC.
