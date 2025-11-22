import math
from move_server.move_logic import PID


def test_zur_mitte_regeln_winkel_0():
    speed, angle = PID.zur_mitte_regeln(0)
    assert speed == PID._linear
    assert angle == 0


def test_zur_mitte_regeln_winkel_pos():
    speed, angle = PID.zur_mitte_regeln(90)
    
    assert speed == PID._linear_langsam
    assert math.isclose(angle, math.pi / 2, rel_tol=1e-6)


def test_zur_mitte_regeln_winkel_neg():
    speed, angle = PID.zur_mitte_regeln(-45)

    assert speed == PID._linear_langsam
    assert math.isclose(angle, -math.pi / 4, rel_tol=1e-6)
