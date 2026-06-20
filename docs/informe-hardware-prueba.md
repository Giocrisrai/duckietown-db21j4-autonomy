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
  PCA9685 `0x40`/`0x70` (motores), IMU `0x68`, ToF `0x29`, OLED `0x3c`.
- **Cámara CSI** (IMX219): `dmesg` muestra `imx219_board_setup: error during i2c
  read probe (-121)` → `/dev/video*` no existe. (Flex CSI mal contactado —
  problema independiente y mecánico.)

## Lo que funciona (descarta varias causas)
- Jetson bootea estable; **no se apaga** con la batería cargada (los apagones
  previos eran por batería baja, ya resueltos).
- MCU/Duckiebattery (`04d8:ecb7`) enumera por USB; `/dev/ttyACM0` presente.
- Bus I2C 0 limpio; bus 2 con sus EEPROMs (`0x50`, `0x57`). El controlador I2C
  del Jetson funciona en los otros buses.
- Stack de software completo y validado (200 tests en host; la suite corre también
  en contenedor en la propia Jetson, monitor de salud por red activo).

## Causas descartadas con evidencia
| Causa | Prueba | Resultado |
|---|---|---|
| Software / driver | `unbind`+`bind` de `tegra-i2c` en `7000c400.i2c` | bus 1 **sigue colgado** |
| Energía | batería cargada + fuente externa | bus 1 **sigue colgado**, Jetson estable, sin `under-voltage` en dmesg |
| Asentado / montaje | múltiples reasentados + inspección del header | bus 1 **sigue colgado**, sin cambio |

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
- **Cámara**: reasentar el flex CSI (contactos al lado correcto, a fondo). El
  sensor (IMX219) y el overlay están bien; el `-121` es solo contacto.

## Estado del software (sin bloqueo)
El stack está listo y subido al repo. En cuanto el bus 1 responda y aparezca
`0x40`, queda inmediato: prueba de motores (`tools/test_motors.py` /
`control/motor_test.py`), sensores (`tools/test_sensors.py`) y calibración fina
(`tools/calibrate.py`).
