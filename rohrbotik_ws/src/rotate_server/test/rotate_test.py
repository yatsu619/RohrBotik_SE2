# test_rotate_cl500.py
import math
import pytest

from rotate_server.rotate_logic import RotateCL500  


def test_normalize_angle_within_range():
    # Winkel im Bereich [-pi, pi] bleiben unverändert (bis auf Rundungsfehler)
    for angle in [-math.pi, -1.0, 0.0, 1.0, math.pi]:
        assert RotateCL500.normalize_angle(angle) == pytest.approx(angle)

def test_normalize_angle_greater_than_pi():
    angle = math.pi + 0.5
    norm = RotateCL500.normalize_angle(angle)
    # Ergebnis muss im Bereich [-pi, pi] liegen
    assert -math.pi <= norm <= math.pi
    # Spezifisch: pi + 0.5 -> -pi + 0.5
    assert norm == pytest.approx(-math.pi + 0.5)

def test_normalize_angle_less_than_minus_pi():
    angle = -math.pi - 0.5
    norm = RotateCL500.normalize_angle(angle)
    assert -math.pi <= norm <= math.pi
    # Spezifisch: -pi - 0.5 -> pi - 0.5
    assert norm == pytest.approx(math.pi - 0.5)

def test_rotate_to_pipe_initial_call_sets_target():
    current_pose = 0.0
    zaehler = 0
    angle_to_target = None

    v_lin, v_ang, ziel_erreicht, new_target = RotateCL500.rotate_to_pipe(
        current_pose, zaehler, angle_to_target
    )

    # 180° in rad = pi
    assert new_target == pytest.approx(math.pi)
    assert v_lin == pytest.approx(0.0)
    # solange nicht im Threshold, soll mit _omega gedreht werden
    assert v_ang == pytest.approx(RotateCL500._omega)
    assert ziel_erreicht is False

def test_rotate_to_pipe_reached_target():
    # Wir tun so, als wäre current_pose schon fast bei target
    target = math.pi
    current_pose = target - 0.005  # kleiner als threshold-Differenz
    zaehler = 1  # Ziel ist schon gesetzt

    v_lin, v_ang, ziel_erreicht = RotateCL500.rotate_to_pipe(
        current_pose, zaehler, target
    )[:3]  # erstes, zweites, drittes Element

    assert ziel_erreicht is True
    assert v_lin == pytest.approx(0.0)
    assert v_ang == pytest.approx(0.0)

def test_rotate_more_initial_call_sets_new_angle():
    current_pose = 0.0
    zaehler = 0
    new_angle = None

    v_lin, v_ang, ziel_erreicht, target_angle = RotateCL500.rotate_more(
        current_pose, zaehler, new_angle
    )

    expected = 79 * math.pi / 180.0
    assert target_angle == pytest.approx(expected)
    assert v_lin == pytest.approx(0.0)
    assert v_ang == pytest.approx(RotateCL500._omega)
    assert ziel_erreicht is False

def test_rotate_more_reached_new_angle():
    # Angle fast erreicht
    target = 79 * math.pi / 180.0
    current_pose = target - 0.005
    zaehler = 1

    v_lin, v_ang, ziel_erreicht = RotateCL500.rotate_more(
        current_pose, zaehler, target
    )[:3]

    assert ziel_erreicht is True
    assert v_lin == pytest.approx(0.0)
    assert v_ang == pytest.approx(0.0)


def test_rotate_to_marker_turns_positive_direction():
    current_pose = 0.0
    marker_rel = 30.0  # Grad

    v_lin, v_ang, ziel = RotateCL500.rotate_to_marker(current_pose, marker_rel)

    assert ziel is False
    assert v_lin == pytest.approx(0.0)
    # 30° > 0 => positive Drehrichtung
    assert v_ang == pytest.approx(RotateCL500._omega)


def test_rotate_to_marker_turns_negative_direction():
    current_pose = 0.0
    marker_rel = -30.0  # Grad

    v_lin, v_ang, ziel = RotateCL500.rotate_to_marker(current_pose, marker_rel)

    assert ziel is False
    assert v_lin == pytest.approx(0.0)
    # -30° < 0 => negative Drehrichtung
    assert v_ang == pytest.approx(-RotateCL500._omega)


def test_rotate_to_marker_target_reached():
    # Marker direkt vor dem Roboter: relative 0°
    current_pose = 1.0  # egal
    marker_rel = 0.0

    v_lin, v_ang, ziel = RotateCL500.rotate_to_marker(current_pose, marker_rel)

    assert ziel is True
    assert v_lin == pytest.approx(0.0)
    assert v_ang == pytest.approx(0.0)
