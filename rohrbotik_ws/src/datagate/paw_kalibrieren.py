#--------- Imports

import rclpy 
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2.aruco as aruco

# kamera_kalibrierung.py
"""
Kamera-Kalibrierung für Raspberry Pi Camera Rev 1.3
Bestimmt die Brennweite für präzise Distanzmessung
"""


# ==================== EINSTELLUNGEN ====================

MARKER_ID = 114                    # Welchen Marker nutzt du?
MARKER_GROESSE_CM = 10.0         # TODO: Mit Lineal messen!
BEKANNTE_DISTANZ_CM = 100.0      # Marker genau 1 Meter entfernt platzieren!

KAMERA_INDEX = 4                 # Oder 2, 4, /dev/video0
AUFLOESUNG = (640, 480)          # Oder (1280, 720) oder (1920, 1080)

# =======================================================

class KameraKalibrierer:
    def __init__(self):
        # Kamera öffnen
        self.camera = cv.VideoCapture(0)
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, AUFLOESUNG[0])
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, AUFLOESUNG[1])
        
        # ArUco Setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Messwerte sammeln
        self.messungen = []
        
        print("=" * 60)
        print("KAMERA-KALIBRIERUNG")
        print("=" * 60)
        print(f"Marker-ID: {MARKER_ID}")
        print(f"Marker-Größe: {MARKER_GROESSE_CM} cm")
        print(f"Bekannte Distanz: {BEKANNTE_DISTANZ_CM} cm ({BEKANNTE_DISTANZ_CM/100} m)")
        print(f"Auflösung: {AUFLOESUNG[0]}x{AUFLOESUNG[1]}")
        print("=" * 60)
        print("\nANLEITUNG:")
        print("1. Platziere Marker GENAU 1 Meter von der Kamera entfernt")
        print("2. Marker muss frontal zur Kamera zeigen")
        print("3. Drücke LEERTASTE um Messung zu machen (10x)")
        print("4. Drücke ESC zum Beenden\n")
    
    def berechne_marker_groesse(self, ecken):
        """Berechnet durchschnittliche Kantenlänge in Pixel"""
        breite = np.linalg.norm(ecken[0] - ecken[1])
        hoehe = np.linalg.norm(ecken[1] - ecken[2])
        return (breite + hoehe) / 2
    
    def berechne_brennweite(self, pixel_groesse):
        """Berechnet Brennweite aus Pixel-Größe"""
        # Formel: Brennweite = (Pixel_Größe × Distanz) / Echte_Größe
        brennweite = (pixel_groesse * BEKANNTE_DISTANZ_CM) / MARKER_GROESSE_CM
        return brennweite
     
    def run(self):
        """Hauptschleife"""
        
        while True:
            success, frame = self.camera.read()
            if not success:
                print("Fehler beim Lesen der Kamera!")
                break
            
            # Graustufen für ArUco
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            
            # Marker suchen
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            marker_gefunden = False
            
            if ids is not None:
                # Zeichne alle gefundenen Marker
                cv.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Suche unseren spezifischen Marker
                for i, detected_id in enumerate(ids.flatten()):
                    if detected_id == MARKER_ID:
                        marker_gefunden = True
                        
                        ecken = corners[i][0]
                        pixel_groesse = self.berechne_marker_groesse(ecken)
                        brennweite = self.berechne_brennweite(pixel_groesse)
                        
                        # Mittelpunkt berechnen
                        mx = int(np.mean(ecken[:, 0]))
                        my = int(np.mean(ecken[:, 1]))
                        
                        # Info anzeigen
                        cv.circle(frame, (mx, my), 5, (0, 255, 0), -1)
                        cv.putText(frame, f"ID: {MARKER_ID}", (mx+10, my-40),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv.putText(frame, f"Groesse: {pixel_groesse:.1f}px", (mx+10, my-20),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv.putText(frame, f"Brennweite: {brennweite:.1f}", (mx+10, my),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Status anzeigen
            status_color = (0, 255, 0) if marker_gefunden else (0, 0, 255)
            status_text = f"Marker {MARKER_ID} gefunden!" if marker_gefunden else f"Marker {MARKER_ID} NICHT gefunden"
            cv.putText(frame, status_text, (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Anzahl Messungen
            cv.putText(frame, f"Messungen: {len(self.messungen)}/10", (10, 60),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Mittelwert anzeigen wenn Messungen vorhanden
            if len(self.messungen) > 0:
                avg = np.mean(self.messungen)
                cv.putText(frame, f"Durchschnitt: {avg:.1f}", (10, 90),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Anleitung
            cv.putText(frame, "LEERTASTE = Messung | ESC = Beenden", (10, frame.shape[0]-10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Bild anzeigen
            cv.imshow("Kamera-Kalibrierung", frame)
            
            # Tastatur-Input
            key = cv.waitKey(1) & 0xFF
            
            # LEERTASTE: Messung machen
            if key == ord(' '):
                if marker_gefunden:
                    self.messungen.append(brennweite)
                    print(f"Messung {len(self.messungen)}: Brennweite = {brennweite:.2f}")
                    
                    if len(self.messungen) >= 10:
                        self.zeige_ergebnis()
                        break
                else:
                    print("FEHLER: Marker nicht im Bild! Positioniere Marker neu.")
            
            # ESC: Beenden
            elif key == 27:
                if len(self.messungen) > 0:
                    self.zeige_ergebnis()
                break
        
        # Aufräumen
        self.camera.release()
        cv.destroyAllWindows()
    
    def zeige_ergebnis(self):
        """Zeigt finale Kalibrierungs-Ergebnisse"""
        print("\n" + "=" * 60)
        print("KALIBRIERUNG ABGESCHLOSSEN!")
        print("=" * 60)
        
        if len(self.messungen) == 0:
            print("Keine Messungen durchgeführt!")
            return
        
        durchschnitt = np.mean(self.messungen)
        std_abweichung = np.std(self.messungen)
        minimum = np.min(self.messungen)
        maximum = np.max(self.messungen)
        
        print(f"\nAnzahl Messungen: {len(self.messungen)}")
        print(f"Einzelwerte: {[f'{m:.1f}' for m in self.messungen]}")
        print(f"\nDurchschnitt:      {durchschnitt:.2f}")
        print(f"Standardabweichung: {std_abweichung:.2f}")
        print(f"Minimum:           {minimum:.2f}")
        print(f"Maximum:           {maximum:.2f}")
        
        print("\n" + "=" * 60)
        print("FÜGE DIESEN WERT IN DEINEN CODE EIN:")
        print("=" * 60)
        print(f"\nself.KAMERA_BRENNWEITE = {durchschnitt:.0f}")
        print(f"# Kalibriert bei {AUFLOESUNG[0]}x{AUFLOESUNG[1]}")
        print("=" * 60)
        
        # Qualitäts-Check
        if std_abweichung > 20:
            print("\n⚠️  WARNUNG: Hohe Standardabweichung!")
            print("   → Marker war nicht stabil/gerade")
            print("   → Wiederhole Kalibrierung!")
        else:
            print("\n✅ Gute Kalibrierung! Standardabweichung ist niedrig.")


def main():
    kalibrierer = KameraKalibrierer()
    kalibrierer.run()


if __name__ == '__main__':
    main()