"""Test del lazo cerrado IMAGEN->IMAGEN: el mundo renderiza la camara, el cerebro
conduce solo con esa imagen y el modelo cinematico actualiza la pose."""
import math

from sim_closed_loop import run_image_loop


def test_converge_al_centro_desde_offset():
    # Arranca derivado 6 cm y torcido: el lazo (imagen real -> (d,phi) -> PID)
    # debe llevarlo al centro. Si algun signo de la cadena fuese erroneo, diverge.
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200)
    assert abs(r["off"][0]) > 0.05                 # empezo lejos del centro
    assert abs(r["off"][-1]) < 0.02                # termino practicamente centrado
    assert r["max_abs_off"] < 0.15                 # no se disparo en el transitorio


def test_converge_desde_el_otro_lado():
    # Mismo experimento con el signo opuesto -> debe converger igual (simetria).
    r = run_image_loop(off0=-0.06, psi0=-0.10, steps=200)
    assert abs(r["off"][-1]) < 0.02


def test_centrado_se_mantiene():
    # Si arranca centrado y recto, se queda centrado (no introduce deriva).
    r = run_image_loop(off0=0.0, psi0=0.0, steps=120)
    assert r["max_abs_off"] < 0.01
