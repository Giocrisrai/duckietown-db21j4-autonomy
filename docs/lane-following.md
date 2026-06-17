# Seguir línea (visión clásica)

Reutilizamos el `line_detector` de `dt-core` (umbral HSV de blanco/amarillo/rojo)
y el `lane_controller` (PID). No reescribimos código; afinamos parámetros.

## Ajuste para tu pista

1. Lanzar: `roslaunch bringup lane_following.launch veh:=<hostname>`
2. Abrir el dashboard y el visor de segmentos (`line_detector_node`).
3. Si la pista tiene iluminación distinta, ajustar los rangos HSV en el
   `line_detector_node` (parámetros `hsv_white`, `hsv_yellow`, `hsv_red`).
4. Validar que el robot complete una vuelta al loop sin salirse.
