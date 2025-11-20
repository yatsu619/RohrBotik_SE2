
# ********************** Imports ********************* 

import cv2 as cv
import cv2.aruco as aruco
import numpy as np 


# **********************Code und Logic Beschreibung ********************* 

"""
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Kommentar des Verlorenen @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Bis jetzt baut die Logik darauf auf das immer nur ein ArUco marker im bild sein darf... da wir ja nciht mehr die id übergeben. 
Das sollte man der schönheit nochmal überdenken. TODO

"""
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ cam_data_logic.py only @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class VisionProcessor:
    def __init__(self):
        """
        Wichtig, je nach eingestellter Kamera müssen hier die Werte für die Kamera-Konstanten geändert werden!!!
        """
        self.check_grayframe = None
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

#Marker infos
        self.marker_gefunden = False
        self.marker_id = None
        self.marker_mittelpunkt = None #(mit x,y)
        self.marker_distanz = None
        self.marker_groesse_pixel = None
        self.marker_winkel = None

#Konstanten der Rasberry Py Camera REV 1.3 // Sensor OmniVision OV5647

        self.MARKER_GROESSE_CM = 17.5       #TODO:  Große des Markers in CM pyhsisch messen und eintragen! Von Rand zu Rand!
        self.KAMERA_BRENNWEITE = 618        #TODO:  zwischen 500 und 700, muss Kalibriert werden --> def kalibriere_brennweite(self):

        self.KAMERA_Breite_Pixel = 640      #TODO:  Oder je nach Einstellung 1280 oder 1920
        self.KAMERA_Hoehe_Pixel = 480       #TODO:  Oder je nach Einstellung 720 oder 1080
        self.KAMERA_Sichtfeld_GRAD = 53.5   #TODO:  53.5 -> bei 640x480 // 62.2 -> 1280x720 // 62.2 -> 1920x1080 



    def find_ArUco(self, gray_frame):
        """
        Suchen nach einer Spezifischer Marker-ID.

        Prüft alle Marker im Bild und speichert dann gewisse Daten des "wanted-id" - Markers

 @@@@@@@@@@ -> Koordinatensystem des Frames:   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@

                (0,0) ──────────────────────────────► X
                │
                │         Ecke 1 ●────────● Ecke 2
                │                │ ArUco  │
                │                │ Marker │
                │         Ecke 4 ●────────● Ecke 3
                │
                ▼ Y
        
@@@@@@@@@@@ -> Strucktur von detectMarkers(corners):   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        
        corners[i]       # Marker Nummer i (z.B. der erste Marker)
        corners[i][0]    # Die Ecken dieses Markers (wir schneiden eine Dimension weg)
        corners[i][0][0] # Die erste Ecke (oben links)


                    corners = [
                # Erster Marker (z.B. ID 0)
                [
                    [  # Diese extra Klammer kommt von OpenCV's Format
                        [200, 80],   # Ecke 1 (oben links)    → X=200, Y=80
                        [250, 80],   # Ecke 2 (oben rechts)   → X=250, Y=80
                        [250, 130],  # Ecke 3 (unten rechts)  → X=250, Y=130
                        [200, 130]   # Ecke 4 (unten links)   → X=200, Y=130
                    ]
                ],
                # Zweiter Marker (z.B. ID 5)
                [
                    [
                        [100, 200],
                        [150, 200],
                        [150, 250],
                        [100, 250]
                    ]
                ]
            ]

            ids = [[0], [5]]  # IDs der gefundenen Marker
                    
        """
#Resett für erneuten Aufruf der Funktion
        self.marker_gefunden = False
        self.marker_id = None
        self.marker_mittelpunkt = None
        self.marker_distanz = None          # in Metern
        self.marker_winkel = None           # in Grad

        if gray_frame is None:
            return False 
        
        corners, ids, rejected = self.detector.detectMarkers(gray_frame)

        if ids is None:
            return False
        
#suche  
        detected_id = ids 
        print(f"Gefundene Marker: {detected_id}")
        self.marker_gefunden = True
        self.marker_id = int(detected_id)
                
        ecken = corners[0][0]                           # durch [i] [0] holen wir uns aus dem 3dimensionlaen Array nur den ersten Marker und die darin liegenden 4 Ecken [x,y]
                
        self.marker_mittelpunkt = (
            np.mean(ecken[:,0]),        #x            z.B.  ecken = [[200,80], [250,80], [250,130], [200,130]]  --> Nur Spalte 0 und alle Zeilen... also 200 + 250 + 250 + 200 / 4 für den Durchschnittswert auf der X-Achse
            np.mean(ecken[:,1])         #y                
        )

        self.marker_winkel = self._berechne_winkel()    
            
        breite = np.linalg.norm(ecken[0] - ecken[1])        # np.linalg.norm() in PIXELN--> √(x² + y²)  [ausrechnen der länge des Vektors] z.B. --> ecken [0] = [200, 80] (oben links) & ecken [1] = [250, 80] (oben rechts) => [200-250, 80-80] =  [-50, 0] np.linalg.norm(-50, 0) = √(-50² + 0²) = 50 pixel breit
        hoehe = np.linalg.norm(ecken[1] - ecken[2])
        self.marker_kantenlaenge_pixel = (breite + hoehe)/2      # Berechnung der DURCHSCHNITTLICHEN KANTENLÄNGE, da der WÜRFEL dennoch druch zerzerrung ungleiche Werte haben könnte.

        self.marker_distanz = ((self.MARKER_GROESSE_CM * self.KAMERA_BRENNWEITE) / self.marker_kantenlaenge_pixel) / 100.0   # Forml der Physik --> Distanz = (Echte_Größe × Brennweite) / Pixel_Größe   --> die /100 sind für die umrechnug von cm zu m

        return True
            

    

# def horizontal_distanz_crossair2ArUco (--> dafür da um den winkel zwischen Marker_mittelpunkt und ArUcomarkermittelpunkt zu berechnen)  --> wird vermutlich schon in find_marker gemacht
    def _berechne_winkel(self):
        """
        Berechnung des Winkels zwischen dem Bildmittelpunkt und dem Mittepunkt des ArUco-Markers

        Logik: 
            Pixel_abweichung = Anzahld er Pixel zwischen Bildmittelpunkt und Mitte-ArUco-Marker
            Grad_pro_Pixel = Rechnet aus dem Winkel der Linse und der Bildbreite in Pixeln die Gradzahl für ein Pixel aus
            Winkel in Grad = Abweichung in Pixeln mal Grad pro Pixel 
        """
        marker_x = self.marker_mittelpunkt[0]
        bild_mitte_x = self.KAMERA_Breite_Pixel / 2

        pixel_abweichung_x = bild_mitte_x - marker_x

        grad_pro_pixel = self.KAMERA_Sichtfeld_GRAD / self.KAMERA_Breite_Pixel
        winkel_grad = pixel_abweichung_x * grad_pro_pixel

        return winkel_grad

# VSP = VisionProcessor() --> Alternativ zum Manager, kann ich auch innerhalb der klasse einfach eine einzelne Instanz aufrufen und diese per Variable an andere Dateien übergeben.


# def rotation (brauchen wir hier eigentlich nicht)

#   def linear fahrt (brauchen wir hier eigentlich nicht)



#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# *************************** Test Main (alle Nodes sollten am Ende, zentral aus einer Datei gestartet werden) ******************************



