# Informe de hardware — Duckiebot `prueba` (DB21-J4)

Documento de entrega para gestión de reparación / garantía. Resume un fallo de
hardware aislado tras diagnóstico exhaustivo por red. Procedimiento técnico
detallado en [diagnostico-hardware-campo.md](diagnostico-hardware-campo.md).

## Equipo
- Duckiebot **DB21-J4**, Jetson Nano **P3450** sobre HUT Duckietown.
- L4T R32.7 / Ubuntu 18.04. Acceso por USB device-mode (`192.168.55.1`, user `prueba`).

## Fallo observado
- **Bus I2C 1 (header de 40 pines, `7000c400.i2c`) se cuelga**: `i2cdetect -y -r 1`
  hace timeout en `00:`. No aparece ninguno de los periféricos del HUT:
  PCA9685 `0x60` (motores) y `0x40`/`0x70` (LEDs), IMU `0x68`, ToF `0x29`, OLED `0x3c`.
- **Cámara CSI** (IMX219): `dmesg` muestra `imx219_board_setup: error during i2c
  read probe (-121)` → `/dev/video*` no existe. (Flex CSI mal contactado —
  problema independiente y mecánico.)

## Lo que funciona (descarta varias causas)
- Jetson bootea estable; **no se apaga** con la batería cargada (los apagones
  previos eran por batería baja, ya resueltos).
- MCU/Duckiebattery (`04d8:ecb7`) enumera por USB; `/dev/ttyACM0` presente.
- Bus I2C 0 limpio; bus 2 con sus EEPROMs (`0x50`, `0x57`). El controlador I2C
  del Jetson funciona en los otros buses.
- Stack de software completo y validado (393 tests en host; la suite corre también
  en contenedor en la propia Jetson, monitor de salud por red activo).

## Causas descartadas con evidencia
| Causa | Prueba | Resultado |
|---|---|---|
| Software / driver | `unbind`+`bind` de `tegra-i2c` en `7000c000.i2c` y `7000c400.i2c` | buses 0 y 1 **siguen colgados** |
| Energía / batería | **9 h 40 min de uptime sin apagarse**, batería cargada + fuente; `POM_5V_IN` estable, térmicos fríos, sin `under-voltage` | buses **siguen colgados**; energía **descartada definitivamente** |
| Asentado / montaje | múltiples reasentados + inspección del header | sin cambio |

## Re-verificación (tras horas, por si era batería)
A las ~9.7 h de uptime continuo (sin un solo apagón), **ambos buses I2C del header
de 40 pines** (`7000c000` = bus 0, pines 27/28; `7000c400` = bus 1, pines 3/5)
**se cuelgan** — antes solo el bus 1. El fallo **se amplió** a los dos buses del
header. El bus 2 interno (EEPROMs 0x50/0x57) sigue OK, así que los controladores
I2C del Jetson funcionan: el problema es el **acople de 40 pines / la HUT**
(probable común-modo: tierra/alimentación del header intermitente, o un pin
puenteado que arrastra ambos buses). La batería queda **100% descartada**.

## Re-verificación 2026-06-22 (cámara cambiada + firma I2C confirmada)
Tras cambiar el cable USB y **el módulo de cámara**, barrido completo por SSH:

- **Cámara: RESUELTA.** El módulo nuevo enumera (`/dev/video0`, `subdev imx219
  8-0010 bound`) y **captura frames reales** (`nvarguscamerasrc → nvjpegenc`, JPEG
  1280×720 válido, todos los modos del sensor hasta 3264×2464 disponibles). El
  `-121` que queda es solo del puerto CSI 7 **vacío**. Pendiente menor: **enfoque**
  del lente (capturas salen con luz/color pero desenfocadas). Ya **no es bloqueo**.
- **I2C: firma de "stuck-low" confirmada por dmesg.** `tegra-i2c 7000c000.i2c: i2c
  transfer timed out addr: 0x3` → el cuelgue ocurre en la **primera dirección del
  escaneo (0x03)**, antes de poder hablar con ningún dispositivo = **línea SDA/SCL
  tomada en bajo**, no un periférico puntual. Persiste en **ambos** buses del header.
- **Resto del hardware, sano:** L4T R32.7.6 (JetPack 4.6.x, Nov-2024); CPU/carga
  ociosa (load 0.11, MAXN); 2.6 GB RAM libres; disco 16 %; **sin under-voltage/
  throttle**; MCU del HUT vivo en `/dev/ttyACM0`; red USB up.

> Esta firma queda **codificada** en `diagnostics/i2c_bus.py` (`classify_i2c_fault`
> → `fault="stuck_low"`, `software_reset_useful=False`,
> `action="power_cycle_reseat"`). El reset por software del controlador **no libera
> una línea tomada en bajo**: la acción correcta es **corte total de energía +
> reasentar/cambiar la HUT** (ver `recommended_steps`).

## Conclusión
Una línea I2C (SDA/SCL) del **bus 1** queda **tomada físicamente** de forma
persistente, independiente de software, energía y montaje. Diagnóstico: **fallo
de hardware** en una de estas piezas (no determinable sin repuestos para
intercambio ni alimentación aislada del módulo):
1. **Pin doblado / en corto** en el header de 40 pines (SDA o SCL a tierra),
   posible en el módulo Jetson o en el HUT.
2. **Chip I2C del HUT en falla** (PCA9685 / IMU / ToF) sujetando la línea.
3. **HUT defectuosa**.

## Acción recomendada
- Gestionar **revisión/cambio del HUT** (y/o del módulo Jetson) con repuesto.
- Test de localización cuando haya repuesto: con **otra HUT** → si el bus 1 deja
  de colgarse, la HUT vieja estaba fallada; si sigue colgado, el daño está en el
  pin de 40 pines del **módulo Jetson**.
- **Cámara**: ✅ **resuelta** con el módulo nuevo (ver re-verificación 2026-06-22);
  solo resta enfocar el lente. Ya no requiere acción de reparación.

## Estado del software (sin bloqueo)
El stack está listo y subido al repo. En cuanto el bus 1 responda y aparezca
`0x60` (motores), queda inmediato: prueba de motores (`tools/detect_motors.py` /
`control/motor_test.py`), sensores (`tools/detect_sensors.py`) y calibración fina
(`tools/calibrate.py`).
