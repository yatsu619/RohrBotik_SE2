# kamera_kalibrierung_headless.py
"""
Kamera-Kalibrierung OHNE GUI für Raspberry Pi
Nur Terminal-Output, kein cv.imshow()


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# 1. Datei auf Bot kopieren
scp kamera_kalibrierung_headless.py pi@robot-ip:/home/pi/

# 2. SSH auf Bot
ssh pi@robot-ip

# 3. Marker aufstellen (1m entfernt!)

# 4. Programm starten
python3 kamera_kalibrierung_headless.py

# 5. Enter drücken wenn bereit
# → Programm macht automatisch 10 Messungen
# → Zeigt Ergebnis im Terminal

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


Aus setting.json (über rohrbotik_ws)

{
    "python-envs.defaultEnvManager": "ms-python.python:system",
    "python-envs.pythonProjects": [],
    "cmake.sourceDirectory": "/home/pa99/rohrbotik_lokal_ws/RohrBotik_SE2/rohrbotik_ws/src/interfaces"
}


"""

# kamera_kalibrierung_headless.py
"""
Kamera-Kalibrierung OHNE GUI für Raspberry Pi
Nur Terminal-Output, kein cv.imshow()
"""

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import time

# ==================== EINSTELLUNGEN ====================

MARKER_ID = 0                    # Welchen Marker nutzt du?
MARKER_GROESSE_CM = 10.0         # TODO: Mit Lineal messen!
BEKANNTE_DISTANZ_CM = 100.0      # Marker genau 1 Meter entfernt platzieren!

KAMERA_DEVICE = '/dev/video0'    # Standard
AUFLOESUNG = (640, 480)

ANZAHL_MESSUNGEN = 10            # Wie viele Messungen?
PAUSE_ZWISCHEN_MESSUNGEN = 2     # Sekunden zwischen Messungen

# =======================================================

class KameraKalibriererHeadless:
    def __init__(self):
        # Kamera öffnen
        print("Öffne Kamera...")
        self.camera = cv.VideoCapture(KAMERA_DEVICE)
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, AUFLOESUNG[0])
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, AUFLOESUNG[1])
        
        # Prüfe ob Kamera offen
        if not self.camera.isOpened():
            print(f"FEHLER: Kamera {KAMERA_DEVICE} konnte nicht geöffnet werden!")
            exit(1)
        
        print("✓ Kamera erfolgreich geöffnet")
        
        # Tatsächliche Auflösung prüfen
        self.echte_breite = None
        self.echte_hoehe = None
        self._pruefe_aufloesung()
        
        # ArUco Setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Messwerte sammeln
        self.messungen = []
        
        print("\n" + "=" * 60)
        print("KAMERA-KALIBRIERUNG (HEADLESS)")
        print("=" * 60)
        print(f"Marker-ID: {MARKER_ID}")
        print(f"Marker-Größe: {MARKER_GROESSE_CM} cm")
        print(f"Bekannte Distanz: {BEKANNTE_DISTANZ_CM} cm ({BEKANNTE_DISTANZ_CM/100} m)")
        print(f"Gewünschte Auflösung: {AUFLOESUNG[0]}x{AUFLOESUNG[1]}")
        print(f"Tatsächliche Auflösung: {self.echte_breite}x{self.echte_hoehe}")
        
        if (self.echte_breite, self.echte_hoehe) != AUFLOESUNG:
            print("⚠️  WARNUNG: Tatsächliche Auflösung weicht ab!")
        
        print(f"Anzahl Messungen: {ANZAHL_MESSUNGEN}")
        print("=" * 60)
        print("\nANLEITUNG:")
        print("1. Platziere Marker GENAU 1 Meter von der Kamera entfernt")
        print("2. Marker muss frontal zur Kamera zeigen")
        print("3. Programm macht automatisch 10 Messungen")
        print("4. CTRL+C zum Abbrechen\n")
        
        # Warte dass User bereit ist
        print("Drücke ENTER wenn Marker platziert ist...")
        input()
        print("\nStarte Kalibrierung in 3 Sekunden...\n")
        time.sleep(3)
    
    def _pruefe_aufloesung(self):
        """Prüft die tatsächliche Auflösung der Kamera"""
        success, test_frame = self.camera.read()
        
        if success:
            self.echte_hoehe, self.echte_breite = test_frame.shape[:2]
        else:
            print("⚠️  Konnte Auflösung nicht prüfen, nehme Einstellungen an")
            self.echte_breite = AUFLOESUNG[0]
            self.echte_hoehe = AUFLOESUNG[1]
    
    def berechne_marker_groesse(self, ecken):
        """Berechnet durchschnittliche Kantenlänge in Pixel"""
        breite = np.linalg.norm(ecken[0] - ecken[1])
        hoehe = np.linalg.norm(ecken[1] - ecken[2])
        return (breite + hoehe) / 2
    
    def berechne_brennweite(self, pixel_groesse):
        """Berechnet Brennweite aus Pixel-Größe"""
        brennweite = (pixel_groesse * BEKANNTE_DISTANZ_CM) / MARKER_GROESSE_CM
        return brennweite
    
    def mache_messung(self):
        """Eine einzelne Messung durchführen"""
        success, frame = self.camera.read()
        
        if not success:
            print("   ✗ Fehler beim Frame lesen")
            return None
        
        # Frame-Größe anzeigen (nur bei erster Messung)
        if len(self.messungen) == 0:
            h, w = frame.shape[:2]
            print(f"   Frame-Größe: {w}x{h} Pixel")
        
        # Graustufen für ArUco
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        # Marker suchen
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None:
            print("   ✗ Kein Marker gefunden")
            return None
        
        # Suche unseren spezifischen Marker
        for i, detected_id in enumerate(ids.flatten()):
            if detected_id == MARKER_ID:
                ecken = corners[i][0]
                pixel_groesse = self.berechne_marker_groesse(ecken)
                brennweite = self.berechne_brennweite(pixel_groesse)
                
                mittelpunkt = (np.mean(ecken[:, 0]), np.mean(ecken[:, 1]))
                
                print(f"   ✓ Marker gefunden!")
                print(f"     Mittelpunkt: ({mittelpunkt[0]:.1f}, {mittelpunkt[1]:.1f})")
                print(f"     Pixel-Größe: {pixel_groesse:.1f}px")
                print(f"     Brennweite: {brennweite:.1f}")
                
                return brennweite
        
        print(f"   ✗ Marker ID={MARKER_ID} nicht gefunden (andere IDs gesehen: {ids.flatten().tolist()})")
        return None
    
    def run(self):
        """Hauptschleife"""
        
        print("Starte Messungen...\n")
        
        for i in range(ANZAHL_MESSUNGEN):
            print(f"Messung {i+1}/{ANZAHL_MESSUNGEN}:")
            
            brennweite = self.mache_messung()
            
            if brennweite is not None:
                self.messungen.append(brennweite)
                print(f"   → Gespeichert!\n")
            else:
                print(f"   → Übersprungen (Marker nicht sichtbar)\n")
            
            # Warte zwischen Messungen (außer bei letzter)
            if i < ANZAHL_MESSUNGEN - 1:
                print(f"Nächste Messung in {PAUSE_ZWISCHEN_MESSUNGEN} Sekunden...")
                time.sleep(PAUSE_ZWISCHEN_MESSUNGEN)
        
        # Aufräumen
        self.camera.release()
        
        # Ergebnis anzeigen
        self.zeige_ergebnis()
    
    def zeige_ergebnis(self):
        """Zeigt finale Kalibrierungs-Ergebnisse"""
        print("\n" + "=" * 60)
        print("KALIBRIERUNG ABGESCHLOSSEN!")
        print("=" * 60)
        
        if len(self.messungen) == 0:
            print("❌ Keine erfolgreichen Messungen!")
            print("   → Prüfe ob Marker sichtbar ist")
            print("   → Prüfe Marker-ID")
            print("   → Prüfe Beleuchtung")
            return
        
        durchschnitt = np.mean(self.messungen)
        std_abweichung = np.std(self.messungen)
        minimum = np.min(self.messungen)
        maximum = np.max(self.messungen)
        
        print(f"\nErfolgreiche Messungen: {len(self.messungen)}/{ANZAHL_MESSUNGEN}")
        print(f"Einzelwerte: {[f'{m:.1f}' for m in self.messungen]}")
        print(f"\nDurchschnitt:      {durchschnitt:.2f}")
        print(f"Standardabweichung: {std_abweichung:.2f}")
        print(f"Minimum:           {minimum:.2f}")
        print(f"Maximum:           {maximum:.2f}")
        
        print("\n" + "=" * 60)
        print("FÜGE DIESE WERTE IN DEINEN CODE EIN:")
        print("=" * 60)
        print(f"\nself.KAMERA_BRENNWEITE = {durchschnitt:.0f}")
        print(f"self.KAMERA_Breite_Pixel = {self.echte_breite}")
        print(f"self.KAMERA_Hoehe_Pixel = {self.echte_hoehe}")
        print(f"# Kalibriert bei {self.echte_breite}x{self.echte_hoehe}")
        print("=" * 60)
        
        # Qualitäts-Check
        if len(self.messungen) < ANZAHL_MESSUNGEN * 0.7:
            print(f"\n⚠️  WARNUNG: Nur {len(self.messungen)}/{ANZAHL_MESSUNGEN} Messungen erfolgreich!")
            print("   → Marker war oft nicht sichtbar")
            print("   → Wiederhole Kalibrierung mit besserem Setup")
        elif std_abweichung > 20:
            print("\n⚠️  WARNUNG: Hohe Standardabweichung!")
            print("   → Marker war nicht stabil positioniert")
            print("   → Wiederhole Kalibrierung!")
        else:
            print("\n✅ Gute Kalibrierung!")
            print(f"   {len(self.messungen)} erfolgreiche Messungen")
            print(f"   Niedrige Standardabweichung ({std_abweichung:.1f})")


def main():
    try:
        kalibrierer = KameraKalibriererHeadless()
        kalibrierer.run()
    except KeyboardInterrupt:
        print("\n\n⚠️  Abgebrochen durch Benutzer")
    except Exception as e:
        print(f"\n❌ FEHLER: {e}")


if __name__ == '__main__':
    main()
