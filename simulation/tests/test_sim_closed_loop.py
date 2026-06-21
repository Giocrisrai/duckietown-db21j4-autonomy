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


def _omega_medio(r):
    om = r["omega"][len(r["omega"]) // 2:]          # segunda mitad (regimen permanente)
    return sum(om) / len(om)


def test_sigue_curva_sin_salirse():
    # Carril que gira a la izquierda (R~0.83 m): el cerebro lo sigue solo con la
    # imagen, manteniendo el error lateral dentro del carril (media calzada 0.11 m).
    r = run_image_loop(off0=0.0, psi0=0.0, curvature=1.2, steps=200)
    assert r["max_abs_off"] < 0.05                  # no se sale del carril
    assert _omega_medio(r) > 0.2                     # gira a la izquierda siguiendo la curva


def test_curva_opuesta_gira_al_otro_lado():
    # Misma curvatura con signo opuesto -> el cerebro gira al otro lado (simetria).
    r = run_image_loop(off0=0.0, psi0=0.0, curvature=-1.2, steps=200)
    assert r["max_abs_off"] < 0.05
    assert _omega_medio(r) < -0.2


def test_curva_cerrada_se_mantiene_en_carril():
    # Curva apretada de Duckietown (R=0.5 m): aun asi sigue dentro del carril.
    r = run_image_loop(off0=0.0, psi0=0.0, curvature=2.0, steps=200)
    assert r["max_abs_off"] < 0.08


def test_converge_con_ruido_gaussiano():
    # Ruido fuerte del sensor (sigma=25): la apertura morfologica + mediana en la
    # estimacion de pose lo absorben y el lazo sigue convergiendo al centro.
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200, noise_sigma=25, seed=1)
    assert abs(r["off"][-1]) < 0.02
    assert r["max_abs_off"] < 0.10


def test_sigue_curva_con_ruido():
    # Curva + ruido del sensor: sigue dentro del carril.
    r = run_image_loop(off0=0.0, psi0=0.0, curvature=1.2, steps=200,
                       noise_sigma=20, seed=2)
    assert r["max_abs_off"] < 0.06


def test_converge_con_oclusion_parcial():
    # Una banda central tapada (sombra/pato sobre parte de las lineas): el robot
    # usa lo que ve (incl. una sola linea por geometria) y sigue convergiendo.
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200, occlusion=0.12, seed=3)
    assert abs(r["off"][-1]) < 0.03


def test_converge_con_brillo_y_ruido():
    # Sobre-iluminacion + ruido a la vez: sigue convergiendo.
    r = run_image_loop(off0=0.06, psi0=0.10, steps=200, brightness=70,
                       noise_sigma=15, seed=4)
    assert abs(r["off"][-1]) < 0.02
