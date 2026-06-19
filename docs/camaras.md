# Soporte de distintos modelos de camara + recalibracion (el modelo NO es limitante)

El stack se **adapta automaticamente** a la camara conectada: detecta el modelo y
elige el pipeline correcto, con fallback USB. Asi el dispositivo no queda atado a
una sola camara. Registro y pipelines en `object_detection/camera_support.py`.

## Autodeteccion (el modelo no limita)
```python
from object_detection.camera_support import auto_pipeline
modelo, pipeline = auto_pipeline(dmesg=open("/dev/kmsg").read())  # o pasa dmesg/devices
# el resto del stack usa `pipeline` sin saber que modelo es; se adapta solo.
```
- Detecta IMX219/IMX219-B (ambas como `imx219`) e IMX477 desde `dmesg`.
- Si no hay sensor CSI pero hay `/dev/video*`, usa **USB/UVC**.
- Si no hay ninguna, lanza un error claro (revisar overlay o conectar USB).

## Agregar un modelo nuevo (extensible)
Anade una entrada al dict `CAMERAS` en `camera_support.py` (tipo csi/v4l2,
resolucion, fps) y, si es CSI, su nombre de sensor a `_CSI_SENSORS`. Nada mas:
pipeline, autodeteccion e intrinsecos por defecto quedan disponibles.

## Camaras soportadas
| id | tipo | resolucion def. | nota |
|---|---|---|---|
| `imx219`   | CSI  | 1280x720 | Sony IMX219 (estandar Duckietown / RPi Cam v2) |
| `imx219-b` | CSI  | 1280x720 | Variante de placa del IMX219 (requiere overlay correcto) |
| `imx477`   | CSI  | 1920x1080 | Sony IMX477 (HQ Camera) |
| `usb`      | V4L2 | 640x480 | USB/UVC generica (fallback universal) |

## Integrar la IMX219-B (lo que no pudieron tus amigos)
La IMX219-B es una variante de placa del IMX219; el sensor es el mismo, pero el
Jetson necesita el **overlay de device-tree** y el **sensor-id** correctos.

1. Diagnostica que ve el sistema:  `tools/probe_camera.sh`
   - Itera CSI `sensor-id 0/1` y USB, y te dice cual captura.
2. Si la CSI no captura, habilita el overlay del IMX219:
   `sudo /opt/nvidia/jetson-io/jetson-io.py` -> Configure CSI Connector ->
   Camera IMX219 -> guardar y **reiniciar**.
3. Revisa que el kernel detecte el sensor:  `dmesg | grep -i imx219`
   y los nodos:  `ls /dev/video*`.
4. Si tienes 2 puertos CSI, prueba `sensor-id=1`.
5. **Alternativa garantizada:** una camara **USB/UVC** funciona sin overlay
   (usa el modo `usb` del stack). Asi nunca te quedas sin camara.

Genera el pipeline correcto desde Python:
```python
from object_detection.camera_support import gst_pipeline
print(gst_pipeline("imx219-b", sensor_id=0))   # CSI nvargus
print(gst_pipeline("usb", device="/dev/video0"))  # fallback USB
```

## Recalibracion al cambiar de camara
Cada sensor tiene su distorsion/intrinsecos. Al cambiar de camara, **recalibra**:

1. Punto de partida automatico (intrinsecos iniciales por sensor):
   ```python
   from object_detection.camera_support import default_intrinsics
   intr = default_intrinsics("imx219-b")   # dict valido para recalibrar
   ```
2. Calibracion real con tablero (en el robot) y guarda el YAML; validalo y usalo:
   ```python
   from object_detection.camera_calibration import load_intrinsics, undistort
   intr = load_intrinsics("/data/config/calibrations/camera_intrinsic/<robot>.yaml")
   frame_corregido = undistort(frame, intr)
   ```
3. Verifica salud del frame con `camera_health.check_frame` (detecta camara tapada/negra/baja resolucion).
