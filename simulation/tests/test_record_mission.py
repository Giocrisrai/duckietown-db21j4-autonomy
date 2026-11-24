"""El grabador de la mision no altera el simulador y anota sin tocar el frame."""
import numpy as np
import pytest

import record_mission
from duckietown_planning.autonomy import AutonomousController


def test_graba_un_frame_por_tick_y_la_mision_llega():
    paso_original = AutonomousController.step
    frames, resultado = record_mission.grabar_mision()
    assert resultado["reached"] is True
    assert len(frames) == resultado["ticks"]
    assert resultado["n_stops"] >= 1                 # el pato obliga a frenar
    assert any(orden["state"] == "STOPPED" for _, orden, _ in frames)
    assert AutonomousController.step is paso_original  # el envoltorio se retira


def test_el_envoltorio_se_retira_aunque_la_mision_falle(monkeypatch):
    paso_original = AutonomousController.step

    def explota(*args, **kwargs):
        raise RuntimeError("fallo simulado")

    monkeypatch.setattr(record_mission.sim_mission, "run_mission_image_loop", explota)
    with pytest.raises(RuntimeError):
        record_mission.grabar_mision()
    assert AutonomousController.step is paso_original


def test_anotar_no_modifica_el_frame_original():
    frame = np.full((480, 640, 3), 60, np.uint8)
    copia = frame.copy()
    anotado = record_mission.anotar(frame, {"state": "STOPPED", "v": 0.0, "omega": 0.0},
                                    True, 3, 10)
    assert np.array_equal(frame, copia)
    assert anotado.shape == frame.shape
    assert not np.array_equal(anotado, frame)


def test_escribe_un_gif_animado(tmp_path):
    pytest.importorskip("PIL")
    from PIL import Image

    frames, _ = record_mission.grabar_mision()
    ruta, n = record_mission.escribir_gif(frames[:30], tmp_path / "m.gif")
    with Image.open(ruta) as gif:
        assert gif.n_frames == n == 10
