# test/test_vision_processor.py

"""
Test-Suite für den VisionProcessor

Diese Datei testet alle wichtigen Funktionen des VisionProcessors:
- Eingabe-Validierung (fehlerhafte/leere Bilder)
- Marker-Erkennung
- Mehrere Marker (nächster wird gewählt)
- Distanz-Berechnung
- Winkel-Berechnung
- Reset-Funktionalität

Ausführen:
    pytest test/test_vision_processor.py -v
    pytest test/test_vision_processor.py -v --cov=datagates
"""

import pytest
import numpy as np
import cv2
import cv2.aruco as aruco
from datagates.cam_VisionProcessor_logic import VisionProcessor


# ========================================
# HILFS-FUNKTIONEN ZUM TESTBILDER ERSTELLEN
# ========================================

def erstelle_testbild_mit_marker(marker_id, marker_groesse_pixel=150, 
                                  position='mitte', bild_breite=640, bild_hoehe=480):
    """
    Erstellt ein Testbild mit einem ArUco Marker.
    
    Parameter:
        marker_id (int): ID des Markers (z.B. 0, 69)
        marker_groesse_pixel (int): Größe des Markers in Pixeln
        position (str): 'mitte', 'links', 'rechts', 'oben', 'unten'
        bild_breite (int): Breite des Bildes in Pixeln
        bild_hoehe (int): Höhe des Bildes in Pixeln
    
    Rückgabe:
        numpy.ndarray: Graustufen-Bild mit Marker
    """
    # ArUco Dictionary (gleiche wie im VisionProcessor)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    
    # Erstelle den Marker
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_groesse_pixel)
    
    # Erstelle leeres Bild (grau/weiß für bessere Erkennung)
    bild = np.ones((bild_hoehe, bild_breite), dtype=np.uint8) * 200  # Hellgrauer Hintergrund
    
    # Berechne Position basierend auf dem Parameter
    if position == 'mitte':
        y_pos = (bild_hoehe - marker_groesse_pixel) // 2
        x_pos = (bild_breite - marker_groesse_pixel) // 2
    elif position == 'links':
        y_pos = (bild_hoehe - marker_groesse_pixel) // 2
        x_pos = 50  # 50 Pixel vom linken Rand
    elif position == 'rechts':
        y_pos = (bild_hoehe - marker_groesse_pixel) // 2
        x_pos = bild_breite - marker_groesse_pixel - 50  # 50 Pixel vom rechten Rand
    elif position == 'oben':
        y_pos = 50  # 50 Pixel vom oberen Rand
        x_pos = (bild_breite - marker_groesse_pixel) // 2
    elif position == 'unten':
        y_pos = bild_hoehe - marker_groesse_pixel - 50  # 50 Pixel vom unteren Rand
        x_pos = (bild_breite - marker_groesse_pixel) // 2
    else:
        # Fallback: Mitte
        y_pos = (bild_hoehe - marker_groesse_pixel) // 2
        x_pos = (bild_breite - marker_groesse_pixel) // 2
    
    # Setze Marker ins Bild
    bild[y_pos:y_pos+marker_groesse_pixel, x_pos:x_pos+marker_groesse_pixel] = marker_img
    
    return bild


def erstelle_testbild_mit_mehreren_markern(marker_infos):
    """
    Erstellt ein Testbild mit mehreren ArUco Markern.
    
    Parameter:
        marker_infos (list): Liste von Tuples (marker_id, groesse, position)
            Beispiel: [(0, 100, 'links'), (69, 150, 'rechts')]
    
    Rückgabe:
        numpy.ndarray: Graustufen-Bild mit mehreren Markern
    """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    bild = np.ones((480, 640), dtype=np.uint8) * 200  # Hellgrauer Hintergrund
    
    for marker_id, groesse, position in marker_infos:
        marker_img = aruco.generateImageMarker(aruco_dict, marker_id, groesse)
        
        # Positionsberechnung (vereinfacht)
        if position == 'links':
            y_pos = (480 - groesse) // 2
            x_pos = 50
        elif position == 'rechts':
            y_pos = (480 - groesse) // 2
            x_pos = 640 - groesse - 50
        elif position == 'mitte':
            y_pos = (480 - groesse) // 2
            x_pos = (640 - groesse) // 2
        else:
            y_pos = (480 - groesse) // 2
            x_pos = (640 - groesse) // 2
        
        # Setze Marker ins Bild (überschreibt evtl. vorhandene)
        bild[y_pos:y_pos+groesse, x_pos:x_pos+groesse] = marker_img
    
    return bild


# ========================================
# TEST-KLASSE
# ========================================

class TestVisionProcessor:
    """
    Haupt-Test-Klasse für den VisionProcessor.
    
    Jede Test-Methode testet einen bestimmten Aspekt.
    setup_method() wird VOR JEDEM Test automatisch ausgeführt.
    """
    
    def setup_method(self):
        """
        Wird vor JEDEM Test ausgeführt.
        Erstellt einen frischen VisionProcessor für jeden Test.
        """
        self.vp = VisionProcessor()
    
    # ========================================
    # TESTS: EINGABE-VALIDIERUNG
    # ========================================
    
    def test_find_aruco_mit_none_frame(self):
        """
        Test: Was passiert wenn None übergeben wird?
        Erwartet: False zurück, marker_gefunden = False
        """
        result = self.vp.find_ArUco(None)
        
        assert result == False, "None-Frame sollte False zurückgeben"
        assert self.vp.marker_gefunden == False, "marker_gefunden sollte False sein"
        assert self.vp.marker_id is None, "marker_id sollte None sein"
        assert self.vp.marker_mittelpunkt is None, "marker_mittelpunkt sollte None sein"
        assert self.vp.marker_distanz is None, "marker_distanz sollte None sein"
        assert self.vp.marker_winkel is None, "marker_winkel sollte None sein"
    
    def test_find_aruco_mit_leerem_schwarzem_bild(self):
        """
        Test: Schwarzes Bild ohne Marker
        Erwartet: False zurück, kein Marker gefunden
        """
        leeres_bild = np.zeros((480, 640), dtype=np.uint8)
        result = self.vp.find_ArUco(leeres_bild)
        
        assert result == False, "Leeres Bild sollte False zurückgeben"
        assert self.vp.marker_gefunden == False, "Kein Marker sollte gefunden werden"
    
    def test_find_aruco_mit_leerem_weissem_bild(self):
        """
        Test: Weißes Bild ohne Marker
        Erwartet: False zurück, kein Marker gefunden
        """
        weisses_bild = np.ones((480, 640), dtype=np.uint8) * 255
        result = self.vp.find_ArUco(weisses_bild)
        
        assert result == False, "Weißes Bild sollte False zurückgeben"
        assert self.vp.marker_gefunden == False, "Kein Marker sollte gefunden werden"
    
    def test_find_aruco_mit_falschem_bildformat(self):
        """
        Test: Bild mit falscher Größe
        Erwartet: Sollte trotzdem funktionieren (OpenCV ist flexibel)
        """
        kleines_bild = np.zeros((100, 100), dtype=np.uint8)
        result = self.vp.find_ArUco(kleines_bild)
        
        # Sollte False zurückgeben (kein Marker), aber nicht crashen
        assert result == False
    
    # ========================================
    # TESTS: EINZELNER MARKER
    # ========================================
    
    def test_find_aruco_findet_marker_id_0(self):
        """
        Test: Marker ID 0 wird korrekt erkannt
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=150)
        result = self.vp.find_ArUco(test_bild)
        
        assert result == True, "Marker sollte gefunden werden"
        assert self.vp.marker_gefunden == True, "marker_gefunden sollte True sein"
        assert self.vp.marker_id == 0, f"Marker-ID sollte 0 sein, ist aber {self.vp.marker_id}"
    
    def test_find_aruco_findet_marker_id_69(self):
        """
        Test: Marker ID 69 wird korrekt erkannt
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=69, marker_groesse_pixel=150)
        result = self.vp.find_ArUco(test_bild)
        
        assert result == True, "Marker sollte gefunden werden"
        assert self.vp.marker_gefunden == True
        assert self.vp.marker_id == 69, f"Marker-ID sollte 69 sein, ist aber {self.vp.marker_id}"
    
    def test_marker_mittelpunkt_liegt_im_bild(self):
        """
        Test: Der berechnete Mittelpunkt liegt innerhalb der Bildgrenzen
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        x, y = self.vp.marker_mittelpunkt
        
        assert 0 <= x <= 640, f"X-Koordinate {x} liegt außerhalb des Bildes"
        assert 0 <= y <= 480, f"Y-Koordinate {y} liegt außerhalb des Bildes"
    
    def test_marker_kantenlaenge_ist_positiv(self):
        """
        Test: Die berechnete Kantenlänge ist positiv und realistisch
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.marker_kantenlaenge_pixel > 0, "Kantenlänge muss positiv sein"
        # Sollte ungefähr 150 Pixel sein (±20 wegen Verzerrung/Erkennung)
        assert 130 < self.vp.marker_kantenlaenge_pixel < 170, \
            f"Kantenlänge {self.vp.marker_kantenlaenge_pixel} weicht zu stark von 150 ab"
    
    # ========================================
    # TESTS: MEHRERE MARKER
    # ========================================
    
    def test_naechster_marker_wird_gewaehlt_groesser_naeher(self):
        """
        Test: Bei zwei Markern wird der GRÖßERE (= nähere) gewählt
        """
        # Marker ID 69 groß (nah), Marker ID 0 klein (weit)
        test_bild = erstelle_testbild_mit_mehreren_markern([
            (0, 80, 'links'),    # Klein = weit weg
            (69, 160, 'rechts')  # Groß = nah
        ])
        
        result = self.vp.find_ArUco(test_bild)
        
        assert result == True, "Marker sollten gefunden werden"
        assert self.vp.marker_id == 69, \
            f"Näherer Marker (69) sollte gewählt werden, aber {self.vp.marker_id} wurde gewählt"
        assert self.vp.marker_kantenlaenge_pixel > 140, "Größerer Marker sollte gewählt sein"
    
    def test_naechster_marker_wird_gewaehlt_umgekehrt(self):
        """
        Test: Gleicher Test, aber Positionen vertauscht
        """
        # Marker ID 0 groß (nah), Marker ID 69 klein (weit)
        test_bild = erstelle_testbild_mit_mehreren_markern([
            (0, 160, 'links'),   # Groß = nah
            (69, 80, 'rechts')   # Klein = weit
        ])
        
        result = self.vp.find_ArUco(test_bild)
        
        assert result == True
        assert self.vp.marker_id == 0, \
            f"Näherer Marker (0) sollte gewählt werden, aber {self.vp.marker_id} wurde gewählt"
    
    def test_drei_marker_naechster_wird_gewaehlt(self):
        """
        Test: Bei drei Markern wird der größte gewählt
        """
        test_bild = erstelle_testbild_mit_mehreren_markern([
            (0, 60, 'links'),     # Klein
            (69, 180, 'mitte'),   # Am größten → sollte gewählt werden
            (5, 100, 'rechts')    # Mittel
        ])
        
        result = self.vp.find_ArUco(test_bild)
        
        assert result == True
        assert self.vp.marker_id == 69, f"Größter Marker sollte gewählt werden"
    
    # ========================================
    # TESTS: WINKEL-BERECHNUNG
    # ========================================
    
    def test_winkel_marker_in_bildmitte(self):
        """
        Test: Marker in der Bildmitte → Winkel sollte ~0° sein
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, position='mitte')
        self.vp.find_ArUco(test_bild)
        
        # Winkel sollte sehr nah an 0 sein (±3° Toleranz)
        assert abs(self.vp.marker_winkel) < 3, \
            f"Winkel sollte ~0° sein, ist aber {self.vp.marker_winkel:.2f}°"
    
    def test_winkel_marker_links_positiv(self):
        """
        Test: Marker links → positiver Winkel
        (Pixel-Abweichung positiv → Winkel positiv)
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, position='links')
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.marker_winkel > 5, \
            f"Winkel sollte deutlich positiv sein (>5°), ist aber {self.vp.marker_winkel:.2f}°"
    
    def test_winkel_marker_rechts_negativ(self):
        """
        Test: Marker rechts → negativer Winkel
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, position='rechts')
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.marker_winkel < -5, \
            f"Winkel sollte deutlich negativ sein (<-5°), ist aber {self.vp.marker_winkel:.2f}°"
    
    def test_winkel_berechnung_konsistent(self):
        """
        Test: Gleiche Position → gleicher Winkel
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, position='links')
        
        # Erster Aufruf
        self.vp.find_ArUco(test_bild)
        winkel1 = self.vp.marker_winkel
        
        # Zweiter Aufruf mit gleichem Bild
        self.vp.find_ArUco(test_bild)
        winkel2 = self.vp.marker_winkel
        
        assert abs(winkel1 - winkel2) < 0.1, "Winkel sollte konsistent sein"
    
    # ========================================
    # TESTS: DISTANZ-BERECHNUNG
    # ========================================
    
    def test_distanz_ist_positiv(self):
        """
        Test: Distanz muss immer positiv sein
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.marker_distanz > 0, \
            f"Distanz muss positiv sein, ist aber {self.vp.marker_distanz}"
    
    def test_distanz_groesserer_marker_kleiner_distanz(self):
        """
        Test: Größerer Marker im Bild → kleinere Distanz (näher)
        """
        # Großer Marker (sollte als nah erkannt werden)
        bild_nah = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=200)
        self.vp.find_ArUco(bild_nah)
        distanz_nah = self.vp.marker_distanz
        
        # Kleiner Marker (sollte als weit erkannt werden)
        bild_fern = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=100)
        self.vp.find_ArUco(bild_fern)
        distanz_fern = self.vp.marker_distanz
        
        assert distanz_nah < distanz_fern, \
            f"Größerer Marker sollte näher sein: nah={distanz_nah:.2f}m, fern={distanz_fern:.2f}m"
    
    def test_distanz_bekannte_marker_groesse_id_0(self):
        """
        Test: Marker ID 0 nutzt korrekte Größe (17.5 cm)
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        # Prüfe dass die richtige Größe verwendet wird
        assert self.vp.MARKER_GROESSE_CM == 17.5, \
            f"Marker ID 0 sollte 17.5cm haben, hat aber {self.vp.MARKER_GROESSE_CM}cm"
    
    def test_distanz_bekannte_marker_groesse_id_69(self):
        """
        Test: Marker ID 69 nutzt korrekte Größe (7.5 cm)
        """
        test_bild = erstelle_testbild_mit_marker(marker_id=69, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.MARKER_GROESSE_CM == 7.5, \
            f"Marker ID 69 sollte 7.5cm haben, hat aber {self.vp.MARKER_GROESSE_CM}cm"
    
    def test_distanz_unbekannte_marker_id_nutzt_standard(self):
        """
        Test: Unbekannte Marker-ID nutzt Standard-Größe
        """
        # Marker ID 99 ist nicht definiert
        test_bild = erstelle_testbild_mit_marker(marker_id=99, marker_groesse_pixel=150)
        self.vp.find_ArUco(test_bild)
        
        assert self.vp.MARKER_GROESSE_CM == 17.5, \
            f"Unbekannte ID sollte Standard 17.5cm nutzen, nutzt aber {self.vp.MARKER_GROESSE_CM}cm"
    
    # ========================================
    # TESTS: RESET-FUNKTIONALITÄT
    # ========================================
    
    def test_reset_bei_erneutem_aufruf_mit_marker(self):
        """
        Test: Werte werden bei erneutem Fund überschrieben
        """
        # Erster Marker
        bild1 = erstelle_testbild_mit_marker(marker_id=0, marker_groesse_pixel=100)
        self.vp.find_ArUco(bild1)
        id1 = self.vp.marker_id
        
        # Zweiter Marker (andere ID, andere Größe)
        bild2 = erstelle_testbild_mit_marker(marker_id=69, marker_groesse_pixel=150)
        self.vp.find_ArUco(bild2)
        id2 = self.vp.marker_id
        
        assert id1 != id2, "IDs sollten unterschiedlich sein"
        assert self.vp.marker_id == 69, "Neue ID sollte übernommen werden"
    
    def test_reset_bei_erneutem_aufruf_ohne_marker(self):
        """
        Test: Alte Werte werden gelöscht wenn kein Marker mehr gefunden wird
        """
        # Erster Aufruf: Marker vorhanden
        bild_mit_marker = erstelle_testbild_mit_marker(marker_id=0)
        self.vp.find_ArUco(bild_mit_marker)
        
        assert self.vp.marker_gefunden == True
        assert self.vp.marker_id is not None
        
        # Zweiter Aufruf: Kein Marker
        leeres_bild = np.zeros((480, 640), dtype=np.uint8)
        self.vp.find_ArUco(leeres_bild)
        
        # ALLE Werte müssen zurückgesetzt sein
        assert self.vp.marker_gefunden == False, "marker_gefunden sollte False sein"
        assert self.vp.marker_id is None, "marker_id sollte None sein"
        assert self.vp.marker_mittelpunkt is None, "marker_mittelpunkt sollte None sein"
        assert self.vp.marker_distanz is None, "marker_distanz sollte None sein"
        assert self.vp.marker_winkel is None, "marker_winkel sollte None sein"


# ========================================
# WENN DIESE DATEI DIREKT AUSGEFÜHRT WIRD
# ========================================

if __name__ == '__main__':
    """
    Erlaubt direktes Ausführen der Tests:
    python test/test_vision_processor.py
    """
    pytest.main([__file__, '-v', '--tb=short'])