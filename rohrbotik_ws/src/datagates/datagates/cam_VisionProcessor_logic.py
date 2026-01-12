# ********************** Imports ********************* 

import cv2 as cv
import cv2.aruco as aruco
import numpy as np 

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class VisionProcessor:
    def __init__(self):
        """
        Die Klasse VisionProcessor ist das Gehirn der Kameralogik. 
        Hier werden in den Instanzvariablen die wichtigsten Informationen der gefunden Marker, als auch Konstanten bezüglich der Kamera und Physischer Markergrößen gespeichert.
        Die eigentiche Auswertung passiert in der Funkion "find_ArUco"
        """
        self.check_grayframe = None
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

#Marker infos
        self.marker_gefunden = False
        self.marker_id = None
        self.marker_mittelpunkt = None 
        self.marker_distanz = None
        self.marker_groesse_pixel = None
        self.marker_winkel = None

#Konstanten der Rasberry Py Camera REV 1.3 // Sensor OmniVision OV5647

        self.MARKER_GROESSE_CM_STANDARD = 17.5       
        self.MARKER_GROESSEN = {
            0:17.5,
            69:7.5,                                   
        }
        
        self.KAMERA_BRENNWEITE = 618        

        self.KAMERA_Breite_Pixel = 640      
        self.KAMERA_Hoehe_Pixel = 480       
        self.KAMERA_Sichtfeld_GRAD = 53.5    



    def find_ArUco(self, gray_frame):
        """
        Die Funktion wird in der cam_subpub_Node aufgerufen und das aktuellste Graubild wird übergeben. 

        Das Bild wird nach ArUco Marker durchsucht und gibt die relevantesten Informationen des NAHELIEGENSTEN Marker aus.

        Funktion/Ablauf: 

        Bei Aufruf der Funktion werden alle Instanzvariablen wieder auf 0 gesetzt.
        Über .detectMarkers werden alle gesichteten Marker Ids sowie deren Eckpunkte gespeichert.

        Nun werden in einer For-Schleife für alle im Ids-index vorliegenden Marker, erst die Pixel-Seitenlänge und dann die Distanz in Metern von Kamera zum physischen Marker berechnet. 
        Der der kamera naheliegenste Marker wird weiter analysiert. Der Mittelpunkt des Markers im Bild sowie der Winkel werden berechnet und in die Instanzvariablen geschrieben.

        Nur wenn mindestens ein Marker gefunden wurde, gibt die Funktion True zurück.
        Die in den Instanzvariablen gesamelten Informationen werden durch die Vererbung an die cam_subpub_Node und von dort per Custom-Message an die Actionserver weiter gegeben.

        
        
        Visuelle Zusatzinfos bezüglich der detectMarkers:

            -> Koordinatensystem des Frames:   

                (0,0) ──────────────────────────────► X
                │
                │         Ecke 1 ●────────● Ecke 2
                │                │ ArUco  │
                │                │ Marker │
                │         Ecke 4 ●────────● Ecke 3
                │
                ▼ Y
        
            -> Strucktur von detectMarkers(corners):  

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

        nearest_marker_id = 0
        nearest_marker_distanz = 999.0
        nearest_marker_index = 0

        for i in range(len(ids)):
            ecken = corners[i][0]

            breite = np.linalg.norm(ecken[0] - ecken[1])        # np.linalg.norm() in PIXELN--> √(x² + y²)  [ausrechnen der länge des Vektors]
            hoehe = np.linalg.norm(ecken[1] - ecken[2])
            sidelength = (breite + hoehe)/2                     # DURCHSCHNITTLICHEN KANTENLÄNGE, da zerzerrung möglich


            marker_probe_id = int(ids[i])
            self.MARKER_GROESSE_CM = self.MARKER_GROESSEN.get(marker_probe_id, self.MARKER_GROESSE_CM_STANDARD )
            
            marker_probe_distanz = ((self.MARKER_GROESSE_CM * self.KAMERA_BRENNWEITE) / sidelength) / 100.0   # Forml der Physik --> Distanz = (Echte_Größe × Brennweite) / Pixel_Größe   --> die /100 sind für die umrechnug von cm zu m

            if marker_probe_distanz < nearest_marker_distanz:
                nearest_marker_distanz = marker_probe_distanz
                nearest_marker_id = marker_probe_id
                nearest_marker_index = i
            

        self.marker_distanz = nearest_marker_distanz

        self.marker_id = nearest_marker_id
        print(f"Gefundenen Marker: {nearest_marker_id}")
        self.marker_gefunden = True

        ecken = corners[nearest_marker_index][0]

        self.marker_mittelpunkt = (
            np.mean(ecken[:,0]),        #x            z.B.   200 + 250 + 250 + 200 / 4 für den Durchschnittswert auf der X-Achse
            np.mean(ecken[:,1])         #y                
        )

        self.marker_winkel = self._berechne_winkel() 

        return True
            


    

# def horizontal_distanz_crossair2ArUco (--> dafür da um den winkel zwischen Marker_mittelpunkt und ArUcomarkermittelpunkt zu berechnen)  --> wird vermutlich schon in find_marker gemacht
    def _berechne_winkel(self):
        """
        Berechnung des Winkels zwischen dem Bildmittelpunkt und dem Mittepunkt des ArUco-Markers

        Logik: 
            Pixel_abweichung = Anzahld er Pixel zwischen Bildmittelpunkt und Mitte-ArUco-Marker
            Grad_pro_Pixel = Rechnet aus dem Winkel der Linse und der Bildbreite in Pixeln die Gradzahl für ein Pixel aus
            Winkel in Grad = Abweichung in Pixeln mal Grad pro Pixel. 
            Bei einem Positiven Winkel dreht sich der Bot richtung Links 
        """
        marker_x = self.marker_mittelpunkt[0]
        bild_mitte_x = self.KAMERA_Breite_Pixel / 2

        pixel_abweichung_x = bild_mitte_x - marker_x                                

        grad_pro_pixel = self.KAMERA_Sichtfeld_GRAD / self.KAMERA_Breite_Pixel
        winkel_grad = pixel_abweichung_x * grad_pro_pixel

        return winkel_grad
    