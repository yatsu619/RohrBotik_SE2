"""
Tests für VisionProcessor
Im Terminal:  "python3 -m pytest src/distance_pub/test/distance_pub_test.py -v"
"""
import pytest
import numpy as np

from datagates.cam_VisionProcessor_logic import VisionProcessor


@pytest.fixture
def vp():
    """Erstellt eine frische VisionProcessor Instanz"""
    return VisionProcessor()


def test_init_default_values(vp):
    """Startwerte sind korrekt gesetzt"""
    assert vp.marker_gefunden == False
    assert vp.marker_id == None
    assert vp.marker_distanz == None
    assert vp.KAMERA_Breite_Pixel == 640
    assert vp.KAMERA_Hoehe_Pixel == 480


def test_find_aruco_none_frame(vp):
    """None als Frame gibt False zurück"""
    result = vp.find_ArUco(None)
    assert result == False


def test_find_aruco_empty_frame(vp):
    """Leeres Bild ohne Marker gibt False zurück"""
    empty_frame = np.zeros((480, 640), dtype=np.uint8)
    result = vp.find_ArUco(empty_frame)
    assert result == False
    assert vp.marker_gefunden == False


def test_marker_groessen_dict(vp):
    """Marker-Größen Dictionary enthält richtige Werte"""
    assert vp.MARKER_GROESSEN[0] == 17.5
    assert vp.MARKER_GROESSEN[69] == 7.5
    assert vp.MARKER_GROESSE_CM_STANDARD == 17.5


def test_berechne_winkel_mitte(vp):
    """Marker in Bildmitte ergibt Winkel ~0"""
    vp.marker_mittelpunkt = (320, 240)  # Bildmitte
    winkel = vp._berechne_winkel()
    assert abs(winkel) < 0.1  # Fast 0


def test_berechne_winkel_links(vp):
    """Marker links von Mitte ergibt positiven Winkel"""
    vp.marker_mittelpunkt = (100, 240)  # Links
    winkel = vp._berechne_winkel()
    assert winkel > 0  # Positiv = nach links drehen