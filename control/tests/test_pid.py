"""Tests del controlador PID."""
from pid import PID


def test_proporcional():
    pid = PID(0.5, 0.0, 0.0)
    assert pid.step(2.0, 0.1) == 1.0           # kp*error


def test_integral_acumula():
    pid = PID(0.0, 1.0, 0.0)
    assert pid.step(1.0, 1.0) == 1.0           # integral=1
    assert pid.step(1.0, 1.0) == 2.0           # integral=2


def test_derivativo_cero_en_primer_paso():
    pid = PID(0.0, 0.0, 1.0)
    assert pid.step(5.0, 0.1) == 0.0           # sin error previo


def test_limite_de_salida():
    pid = PID(10.0, 0.0, 0.0, out_limits=(-1.0, 1.0))
    assert pid.step(100.0, 0.1) == 1.0         # clamp


def test_reset_limpia_integral():
    pid = PID(0.0, 1.0, 0.0)
    pid.step(1.0, 1.0)
    pid.reset()
    assert pid.step(1.0, 1.0) == 1.0           # integral reiniciado


def test_anti_windup_bajo_saturacion_de_actuador():
    # Con la salida saturada por out_limits, el integral NO debe seguir creciendo
    # (windup): integracion condicional ligada a la saturacion del actuador.
    pid = PID(kp=1.0, ki=2.0, kd=0.0, out_limits=(-1.0, 1.0))
    for _ in range(50):
        pid.step(1.0, 0.1)                     # error grande sostenido -> satura en +1
    # Sin anti-windup el integral valdria ~10 (50*1*0.1*ki/...); debe quedar acotado.
    assert pid._integral <= 1.5


def test_reacciona_rapido_tras_saturacion():
    # Tras saturar, al invertirse el error el actuador debe responder casi de
    # inmediato (no quedarse "pegado" en el tope por el integral acumulado).
    pid = PID(kp=1.0, ki=2.0, kd=0.0, out_limits=(-1.0, 1.0))
    for _ in range(20):
        pid.step(1.0, 0.1)                     # satura en +1
    pasos_pegado = 0
    for _ in range(100):
        out = pid.step(-1.0, 0.1)              # error invertido: pedimos salida negativa
        if out >= 0.999:
            pasos_pegado += 1
        else:
            break
    assert pasos_pegado <= 1                    # reacciona en <=1 paso, no ~10
