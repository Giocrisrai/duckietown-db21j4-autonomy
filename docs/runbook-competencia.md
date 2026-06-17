# Runbook de competencia — DB21-J4

## Antes (en casa)
- [ ] `python -m pytest -v` todo verde.
- [ ] Engine `model.engine` construido y copiado a `/data/` del robot.
- [ ] Vuelta completa de seguir línea OK (Task 12).
- [ ] Detección de patos OK a ~10–25 FPS (Task 11).
- [ ] Intersecciones reactivas OK (Task 14).
- [ ] Batería cargada; SD respaldada.

## En el venue
1. Encender robot, `tools/check_robot.sh <hostname>` → `[OK]`.
2. Ajustar HSV a la luz del lugar si el carril falla (docs/lane-following.md).
3. Si conoces el trazado: armar `planning/maps/<pista>.yaml` y lanzar con
   `use_global_planner:=true start:=<...> goal:=<...>`.
4. Si NO lo conoces: dejar `use_global_planner:=false` (reactivo).

## Fallbacks en cascada (de más a menos funcionalidad)
- Falla el planeador global → `use_global_planner:=false` (reactivo siempre funciona).
- Falla YOLO/TensorRT → comentar el include de `object_detection` en `full_stack.launch`
  y correr solo seguir línea + intersecciones.
- Falla una intersección → bajar velocidad (`lane_controller` `v_bar`) y reintentar.
- Todo lo demás falla → `lane_following.launch` solo (mínimo: dar vueltas al loop).

## Comando de arranque
roslaunch bringup full_stack.launch veh:=<hostname> use_global_planner:=<true|false>
