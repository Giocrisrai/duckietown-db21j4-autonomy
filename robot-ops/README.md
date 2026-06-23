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

## Seguridad (importante)
Estos scripts asumen el **enlace USB device-mode privado** (`192.168.55.x`, robot↔host,
no expuesto a internet). Endurecimiento aplicado y recomendaciones:

- **Bind acotado:** x11vnc escucha **solo en la IP del enlace USB** (`-listen
  192.168.55.1`), no en `0.0.0.0` → no se expone en otras interfaces (WiFi/Ethernet).
- **Sin clave débil por defecto:** `setup_vnc_nosudo.sh` **exige** una clave fuerte
  como argumento; `~/.vnc/passwd` queda con permisos `600`.
- **VNC va en CLARO (sin cifrar).** Solo es aceptable en el enlace USB privado. En
  **cualquier red no confiable (WiFi/Ethernet)**: NO expongas el 5900 — usa **túnel
  SSH** y bind a localhost:
  ```bash
  # host:
  ssh -L 5900:localhost:5900 prueba@<ip-robot>
  # robot:  arranca con BIND=localhost
  bash robot-ops/setup_vnc_nosudo.sh 'CLAVE-FUERTE' localhost
  # luego conecta el cliente VNC a  vnc://localhost:5900
  ```
- **Auto-arranque:** `vnc_autostart.sh` deja el VNC corriendo en cada boot. Si no lo
  quieres permanente, quita la línea `@reboot` del `crontab -e`.
- **NOPASSWD sudo de debug:** si habilitaste `/etc/sudoers.d/prueba-nopw` para
  configurar la resolución, **revértelo al terminar** (vuelve a pedir contraseña):
  ```bash
  sudo rm /etc/sudoers.d/prueba-nopw
  ```
