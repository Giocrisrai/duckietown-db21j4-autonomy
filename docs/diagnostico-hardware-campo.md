# Diagnóstico de hardware en campo — DB21-J4 (caso real)

Registro de un diagnóstico real sobre el Duckiebot `prueba` (Jetson Nano P3450
sobre HUT DB21-J4), accedido por red (USB device-mode, `192.168.55.1`). Sirve
como procedimiento reproducible cuando "los motores/sensores/cámara no aparecen".

## Síntoma

- Motores, IMU y ToF no responden; la cámara no entrega imagen.
- `i2cdetect -y -r 1` (bus del HUT) **se cuelga** (timeout, se queda en `00:`).
- `/dev/video*` no existe.

## Mapa de buses I2C del Jetson Nano (Tegra X1)

| /dev | Controlador | Uso |
|---|---|---|
| i2c-0 | `7000c000.i2c` | header 40-pin, pines 27/28 |
| **i2c-1** | **`7000c400.i2c`** | **header 40-pin, pines 3/5 → HUT Duckietown** |
| i2c-2 | `7000c500.i2c` | EEPROMs de placa/cámara (0x50, 0x57) |
| i2c-7 / i2c-8 | mux CSI | I2C de control de la cámara CSI |

Direcciones esperadas en un HUT sano (bus 1) — el HATv3 lleva DOS PCA9685:
`0x60` (PCA9685 de **motores**), `0x40` + `0x70` (PCA9685 de **LEDs** + su all-call),
`0x68` (IMU MPU-6050), `0x29` (ToF VL53L0X), `0x3c` (OLED).

## Procedimiento de aislamiento (de software a hardware)

Ejecutar en orden; cada paso descarta una causa.

### 1. ¿La HUT está alimentada y comunica por USB?
```bash
lsusb | grep -i 04d8        # MCU/Duckiebattery (04d8:ecb7) presente?
ls /dev/ttyACM*             # /dev/ttyACM0 -> MCU enumerado
```
El MCU/batería usa **USB** (no el header de 40 pines). Si aparece, la HUT tiene
energía y su enlace USB funciona — pero eso **no** garantiza el I2C de 40 pines.

### 2. ¿El bus se cuelga? Clasificar ANTES de actuar (leer dmesg)
La acción correcta depende de **dónde** hace timeout el bus. Leer el dmesg:
```bash
dmesg | grep -iE 'tegra-i2c.*timed out'
#   ej: tegra-i2c 7000c400.i2c: i2c transfer timed out addr: 0x3
```
- **Timeout en `addr: 0x3`** (la primera dirección del escaneo) → la línea SDA/SCL
  está **tomada en bajo** *antes* de poder hablar con nada. **El reset por software
  NO sirve** (resetea el controlador, no la línea). Saltar al paso 3 / inspección
  física. Más aún si cuelgan **ambos** buses del header (común-modo en la HUT).
- **Timeout en una dirección alta**, o bus colgado **sin** línea de timeout →
  probable cuelgue del **controlador**; ahí sí vale el reset por software:
  ```bash
  echo 7000c400.i2c | sudo tee /sys/bus/platform/drivers/tegra-i2c/unbind
  sleep 1
  echo 7000c400.i2c | sudo tee /sys/bus/platform/drivers/tegra-i2c/bind
  sleep 1
  sudo timeout 10 i2cdetect -y -r 1
  ```
  Si responde → era el controlador (resuelto). Si sigue colgado → línea tomada.

> Esta lógica está **codificada y testeada** en `diagnostics/i2c_bus.py`
> (`classify_i2c_fault(bus_status, dmesg)` → `fault`, `software_reset_useful`,
> `action`, `recommended_steps`). Caso `prueba`: `fault="stuck_low"`,
> `software_reset_useful=False`, `action="power_cycle_reseat"`.
>
> **Atajo (un comando, en el robot):** `sudo python3 tools/diagnose_i2c.py`
> escanea ambos buses del header (con `timeout` por si cuelgan), lee `dmesg` y
> imprime el veredicto + pasos automáticamente.

### 3. ¿Es falta de energía? (descarta alimentación)
Alimentar con batería cargada **y** fuente externa, reescanear. Si el bus **sigue
colgado con el Jetson estable** (uptime crece, sin apagones, sin `under-voltage`
en `dmesg`) → **no es energía**.

### 4. Cámara CSI — leer dmesg
```bash
dmesg | grep -iE 'imx219|ov5647|camera|csi'
```
- `imx219 ... tegracam sensor driver` → overlay correcto, sensor reconocido.
- `imx219_board_setup: error during i2c read probe (-121)` → **flex mal
  contactado** (EREMOTEIO). No es overlay ni software: reasentar el flex CSI
  (contactos al lado correcto, a fondo, traba cerrada).

## Conclusión del caso `prueba`

Descartados **software** (reset del controlador no libera el bus), **energía**
(batería + fuente, Jetson estable sin apagones ni undervoltage) y **montaje**
(varios reasentados; el bus pasa de vacío a colgado). El bus 1 queda **tomado
físicamente** → causa en el **header de 40 pines** (pin doblado que puentea SDA/SCL
a tierra) o en un **chip I2C del HUT** (PCA9685/IMU/ToF en corto) o **HUT
defectuosa**. La cámara es IMX219 con `-121` = **flex CSI mal contactado**.

### Inspección física dirigida (lo único que queda)
1. **Corte total de energía**: apagar y **desconectar la batería ~30 s**. Un chip
   I2C latcheado que mantiene SDA/SCL en bajo solo se libera cortando su
   alimentación por completo (un reboot caliente no basta). Retirar el Jetson del HUT.
2. Header de 40 pines con lupa: buscar **pin doblado**, sobre todo en/junto a los
   **pines 3 y 5** (SDA/SCL); verificar que ninguna fila quede corrida (offset).
3. Reasentar **derecho y a ras**; encender una sola vez y reescanear el bus 1.
4. Reasentar el **flex CSI** de la cámara (problema independiente, `-121`).
5. Si con el header bien puesto el bus 1 sigue colgado → probar **otra HUT o
   batería**: el fallo está en la placa/chip, no en el montaje.

> Nota: nada de esto es del software del stack. La suite (324 local; tambien corre en
> contenedor en la Jetson), la calibración base y el monitoreo por red funcionan;
> el bloqueo es puramente el acople físico HUT↔Jetson y el flex de cámara.
