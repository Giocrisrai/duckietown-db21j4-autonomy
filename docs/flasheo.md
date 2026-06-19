# Flashear la Jetson desde la Mac (o cualquier dispositivo)

**Sí se puede flashear desde tu Mac** (o Linux/Windows). La DB21-J4 usa una Jetson
Nano que **arranca desde microSD**, asi que se flashea la **tarjeta SD** (no la
Jetson por USB). Solo necesitas un **lector de microSD** en la Mac.

## Pasos (desde la Mac, sin depender de dts)
1. **Conseguir la imagen base** (una de estas):
   - Imagen de Duckietown para Jetson (JetPack 4.6.x) — la que usa el robot.
   - Imagen oficial **NVIDIA JetPack 4.6** para Jetson Nano (SD card image).
   - Imagen comunitaria (p. ej. Qengineering Ubuntu 20.04 para Nano).
   > Cualquiera sirve si trae **Docker** (y `nvidia-container-runtime` para GPU);
   > nuestro stack corre en contenedores encima, asi que es compatible.
2. **Identificar la SD con seguridad** (para no borrar el disco equivocado):
   `tools/list_disks.sh`   (macOS: diskutil / Linux: lsblk)
3. **Flashear** (dos opciones):
   - **Fácil (GUI, recomendado):** balenaEtcher (Mac/Win/Linux) — selecciona imagen y SD.
   - **CLI (nuestro script):** `tools/flash_sd_raw.sh <imagen.img[.xz]> <dispositivo>`
     - macOS: dispositivo `/dev/rdiskN` (RAW, rapido). Antes: `diskutil unmountDisk /dev/diskN`.
     - Linux: `/dev/sdX` o `/dev/mmcblkN`.
4. **Arrancar** la Jetson con la SD, conectarla a la red (WiFi/Ethernet).
5. **Desplegar nuestro stack** (desde la Mac, por red, sin dts):
   `tools/deploy_robot.sh tar <host> robot.tar`  (construido con `tools/build_image.sh`).
6. **Preflight + monitoreo**: `tools/validate_hardware.sh <host>` y `python -m run_monitor`.

## ¿Y la compatibilidad con lo que montamos?
Nuestro software va **containerizado (Docker)** y es **agnostico de placa**: una vez
la SD tiene un SO con Docker, los contenedores del stack corren igual. Por eso el
flasheo (SO base) y el despliegue (nuestra app) estan **desacoplados** — flasheas
una vez y luego iteras la app por red.

## Caveat: Jetson con eMMC (no es el caso del dev kit / DB21-J4)
Las Jetson de **modulo con eMMC** (sin SD) requieren **NVIDIA SDK Manager** en un
host **Ubuntu x86** via USB recovery — eso NO se hace desde Mac. La DB21-J4 es
**SD-based**, asi que el flasheo desde Mac funciona sin problema.

## Otras placas (Raspberry Pi)
Mismo flujo: flashear Raspberry Pi OS (64-bit) a la SD desde la Mac (Raspberry Pi
Imager o balena/dd) → Docker → `deploy_robot.sh`. Ver docs/hardware-alternativo.md.
