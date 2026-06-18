import pytest
from duckietown_planning.intersection_policy import choose_turn


def test_respeta_el_plan_si_es_valido():
    assert choose_turn(["left", "straight"], planned="left") == "left"


def test_ignora_plan_invalido_y_usa_prioridad():
    # planned=right no esta disponible -> cae a prioridad (straight primero)
    assert choose_turn(["left", "straight"], planned="right") == "straight"


def test_sin_plan_usa_prioridad_por_defecto():
    assert choose_turn(["left", "right"]) == "right"   # straight no esta, right antes que left


def test_sin_salidas_lanza_error():
    with pytest.raises(ValueError):
        choose_turn([])
