"""Graba en un GIF lo que VE el robot durante una mision simulada.

No cambia el simulador: envuelve `AutonomousController.step` mientras corre
`run_mission_image_loop` y guarda cada frame de camara junto con la orden que el
cerebro devolvio para ese frame. Sobre cada imagen se anota el estado de la
maquina de estados, la velocidad y el giro, que es justo lo que el robot decide
solo a partir de esos pixeles.

    make sim-video                       # escribe docs/media/sim-mission.gif
    python -m record_mission salida.gif  # (con PYTHONPATH de las demos)

Requiere Pillow ademas de las dependencias de desarrollo.
"""

import sys
from pathlib import Path

import cv2

import sim_mission
from duckietown_planning.autonomy import AutonomousController
from duckietown_planning.map_graph import MapGraph

# Color (BGR) de la banda de estado segun lo que esta haciendo el cerebro.
_COLORES = {"STOPPED": (40, 40, 220), "TURNING": (220, 140, 30)}
_COLOR_POR_DEFECTO = (60, 170, 60)


def _mapa_demo():
    """El mismo mapa de tres intersecciones que usa la demo `sim-mission`."""
    return MapGraph(
        ["A", "B", "C", "D"],
        [
            {"from": "A", "to": "B", "turn": "right"},
            {"from": "B", "to": "C", "turn": "straight"},
            {"from": "C", "to": "D", "turn": "left"},
        ],
    )


def grabar_mision(obstacle_ticks=range(40, 70), noise_sigma=0, seed=3, **kwargs):
    """Corre la mision A -> D y devuelve (frames, resultado).

    `frames` es una lista de (imagen BGR, orden) por tick, en el orden en que el
    cerebro los proceso. Por defecto hay un pato delante durante 30 ticks, para
    que se vea el frenado por fusion vision+ToF y la reanudacion. Va sin ruido de
    camara porque el ruido hace el GIF incompresible (pasa de ~2 MB a mas de 20).
    """
    frames = []
    paso_original = AutonomousController.step

    def paso_grabado(self, obs, dt=0.05):
        orden = paso_original(self, obs, dt)
        frames.append((obs["image"].copy(), dict(orden), bool(obs.get("detections"))))
        return orden

    AutonomousController.step = paso_grabado
    try:
        resultado = sim_mission.run_mission_image_loop(
            _mapa_demo(), "A", "D", obstacle_ticks=obstacle_ticks,
            noise_sigma=noise_sigma, seed=seed, **kwargs)
    finally:
        AutonomousController.step = paso_original
    return frames, resultado


def anotar(imagen, orden, hay_obstaculo, tick, total):
    """Devuelve una copia del frame con la telemetria del cerebro superpuesta."""
    img = imagen.copy()
    estado = str(orden.get("state", "?"))
    color = _COLORES.get(estado, _COLOR_POR_DEFECTO)
    cv2.rectangle(img, (0, 0), (img.shape[1], 58), (20, 20, 20), -1)
    cv2.rectangle(img, (0, 0), (10, 58), color, -1)
    cv2.putText(img, estado, (22, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2,
                cv2.LINE_AA)
    detalle = "v={:.2f} m/s  omega={:+.2f} rad/s".format(
        float(orden.get("v", 0.0)), float(orden.get("omega", 0.0)))
    if orden.get("turn"):
        detalle += "  turn={}".format(orden["turn"])
    cv2.putText(img, detalle, (22, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1,
                cv2.LINE_AA)
    cv2.putText(img, "tick {}/{}".format(tick, total), (img.shape[1] - 150, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
    if hay_obstaculo:
        cv2.putText(img, "OBSTACLE (vision + ToF)", (img.shape[1] - 250, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 200, 255), 1, cv2.LINE_AA)
    return img


def escribir_gif(frames, salida, cada=3, ancho=420, ms_por_frame=120, colores=48):
    """Escribe el GIF tomando uno de cada `cada` frames, reducido a `ancho` px."""
    from PIL import Image

    total = len(frames)
    imagenes = []
    for i in range(0, total, cada):
        imagen, orden, hay_obstaculo = frames[i]
        anotada = anotar(imagen, orden, hay_obstaculo, i + 1, total)
        alto = int(anotada.shape[0] * ancho / anotada.shape[1])
        chica = cv2.resize(anotada, (ancho, alto), interpolation=cv2.INTER_AREA)
        rgb = Image.fromarray(cv2.cvtColor(chica, cv2.COLOR_BGR2RGB))
        imagenes.append(rgb.quantize(colors=colores, dither=Image.Dither.NONE))
    salida = Path(salida)
    salida.parent.mkdir(parents=True, exist_ok=True)
    imagenes[0].save(salida, save_all=True, append_images=imagenes[1:],
                     duration=ms_por_frame, loop=0, optimize=True)
    return salida, len(imagenes)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    salida = argv[0] if argv else "docs/media/sim-mission.gif"
    frames, resultado = grabar_mision()
    ruta, n = escribir_gif(frames, salida)
    print("mision: llego={} giros={} paradas={} ticks={}".format(
        resultado["reached"], resultado["turns_taken"], resultado["n_stops"], resultado["ticks"]))
    print("gif: {} ({} frames)".format(ruta, n))
    return 0 if resultado["reached"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
