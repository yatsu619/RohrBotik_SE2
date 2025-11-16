
# ********************** Imports ********************* 

import cv2 as cv 
import cv2.aruco as aruco
import numpy as np 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from cam_data_Node import CameraOutPut
import cv2.aruco as aruco


# ********************** Pseudo-Code********************* 

"""
<<<<<<<<<<<<<<<<<<<<<< DONE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def "Bilder empfangen und Bereitstellen"
-> Empfangen der Kamera bilder über ROS2 - Camera-Node
-> Wenn nötig Puffern der Bilder (wenn das programm zum auswerten länger braucht als Bilder ankommen)
-> Verarbeiten der Bilder in openCV format und umwandeln in Graustufen (falls das besser zur erkennung von ArUcomarken ist)
-> Bilder der restlichen Klasse / .py Datei breitstellen
<<<<<<<<<<<<<<<<<<<<<< DONE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


def "and_rutate"(wird von rotate_server angefordert)

Diese Funktion per se benötigt es hier nicht wirklich... die funktionalität den marker mit distanz und id zu erkennen reicht, um zu drehen und stehen zu bleiben. 
Allerdings kann man die Berechnung des Winkels dennoch als eigene Funtion noch mit implementieren, wenn der Bot zu weit über das Ziel hinausdreht und man den Marker aber immer noch sieht

---- Ab hier muss noch überdacht werden ------
def "linear_regelung"(wird von move_server angefordert)

-> Bild von "Bild empfangen und Bereitstellen" einlesen.
-> ArUco-Marker am Ende des Tunnels (platziert am Ende der Strecke) erkennen und anvisieren. 
-> Der Marker soll dann immer in der Mitte des bildes sein. Wenn nicht, muss die Abweichung per Winkel berechnet und in Grad an den move_server logic py übergeben werden. 
--> der Move_server frägt das alle paar cm ab. Wenn eine zu große Abweichung entsteht muss Move_server per ODOM um den berechneten winkel (abweichung) gegenlenken. 


def "stop_it" (wird von move_server abgefragt)
-> Bild von "Bild empfangen und Bereitstellen" einlesen.
-> ArUco-Marker am Ende des Tunnels (platziert am Ende der Strecke) erkennen und anvisieren.
-> Distanz zum End-Marker berechnen. 
-> ist die Distanz kleiner als eine Vordefinierte Strecke (kurz vor dem Schild ist auch der "Stopp-Puntk"), dann soll per BOOL die Info an server übergeben werden und einen Stop auslösen. 
[BONUS: Grundlegend war der Gedanke, dass übergeordnet der Bot IMMER stehen bleibt, sobald kein Aruco-marker bei der Linearfahrt mehr zu sehen ist. Das würde auch gleichzeitig einen Stop bei objekten die im Weg liegen ermöglichen]
"""# du musst nur den bool liefern um die stop logic kümmer ich mich @johannes-wk
"""

(plathalter für die def "traffic" funktion
Diese Funktion soll erkennen ob ein zweiter Roboter oder anderer gegenstand im Rohr im Weg steht und sofort stehen bleiben und nen Fehler werfen. 
Alternative soll er einen anderen Roboter mit Aruco marker sehen und ihm hinterher fahren. ) 
--> Aber das kommt erst später, daher jetzt noch nicht relevant. 
"""

# ********************** Main-Code********************* 

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ ROS2 - Subscriber_cam @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Camera_data(Node):
    def __init__(self):
        super().__init__('cam_data_sub')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'pycam_tb3',
            self.frame_callback,
            10
        )
        self.vipr = VisionProcessor()                     # Hier erstelle ich das Objekt VisionProcessor,um dort die Variable frame zu übergeben.
        
    def frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is not None:
            self.vipr.videoframe(frame)

        else:
            self.get_logger().info("Da knallts bei der Cam gewaltig")




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

        self.MARKER_GROESSE_CM = 10         #TODO:  Große des Markers in CM pyhsisch messen und eintragen! Von Rand zu Rand!
        self.KAMERA_BRENNWEITE = 600        #TODO:  zwischen 500 und 700, muss Kalibriert werden --> def kalibriere_brennweite(self):

        self.KAMERA_Breite_Pixel = 640      #TODO:  Oder je nach Einstellung 1280 oder 1920
        self.KAMERA_Hoehe_Pixel = 480       #TODO:  Oder je nach Einstellung 720 oder 1080
        self.KAMERA_Sichtfeld_GRAD = 53.5   #TODO:  53.5 -> bei 640x480 // 62.2 -> 1280x720 // 62.2 -> 1920x1080 


    def videoframe(self, color_frame):
        self.check_grayframe = cv.cvtColor(color_frame, cv.COLOR_BGR2GRAY)                  # Für besseres Erkennen der ArUco marker, wird das bild in Graustufen unterteilt

    def find_ArUco(self, wanted_id):
        """
        Suchen nach einer Spezifischer Marker-ID.

        Prüft alle Marker im Bild und speichert dann gewisse Daten des "wanted-id" - Markers

        -> Strucktur von detectMarkers(corners):

        corners                     # Liste aller gefundenen Marker
            ├─ corners[0]           # Erster Marker (nochmal verschachtelt wegen OpenCV-Format)
            │   └─ corners[0][0]    # Die 4 Ecken als Array [[x,y], [x,y], [x,y], [x,y]]
            ├─ corners[1]           # Zweiter Marker
            │   └─ corners[1][0]
            └─ ..

        -> Koordinatensystem des Frames:

        (0,0) ────────────────────► X (Breite)
            │
            │    Hier ist dein Bild
            │   Startet OBEN LINKS
            │  
            ▼
            Y (Höhe)

        """
#Resett für erneuten Aufruf der Funktion
        self.marker_gefunden = False
        self.marker_id = None
        self.marker_mittelpunkt = None
        self.marker_distanz = None          # in Metern
        self.marker_winkel = None           # in Grad

        if self.check_grayframe is None:
            return False 
        
        corners, ids, rejected = self.detector.detectMarkers(self.check_grayframe)

        if ids is None:
            return False
        
#suche   
        for i, detected_id in enumerate(ids.flatten()):
            if detected_id == wanted_id:
                self.marker_gefunden = True
                self.marker_id = int(detected_id)
                
                ecken = corners[i][0]                           # durch das [0] schneiden wir eine Dimension des Arrays weg [[[2,1], [2,4]]] --> [[20,23], [24,20]]
                self.marker_mittelpunkt = (
                    np.mean(ecken[:,0]),        #x            z.B.  ecken = [[200,80], [250,80], [250,130], [200,130]]  --> Nur Spalte 0 und alle Zeilen... also 200 + 250 + 250 + 200 / 4 für den Durchschnittswert auf der X-Achse
                    np.mean(ecken[:,1])         #y                
                )

                self.marker_winkel = self._berechne_winkel()    
            
                breite = np.linalg.norm(ecken[0] - ecken[1])        # np.linalg.norm() --> √(x² + y²)  [ausrechnen der länge des Vektors] z.B. --> ecken [0] = [200, 80] (oben links) & ecken [1] = [250, 80] (oben rechts) => [200-250, 80-80] =  [-50, 0] np.linalg.norm(-50, 0) = √(-50² + 0²) = 50 pixel breit
                hoehe = np.linalg.norm(ecken[1] - ecken[2])
                self.marker_kantenlaenge_pixel = (breite + hoehe)/2      # Berechnung der DURCHSCHNITTLICHEN KANTENLÄNGE, da der WÜRFEL dennoch druch zerzerrung ungleiche Werte haben könnte.

                self.marker_distanz = ((self.MARKER_GROESSE_CM * self.KAMERA_BRENNWEITE) / self.marker_kantenlaenge_pixel) / 100.0   # Forml der Physik --> Distanz = (Echte_Größe × Brennweite) / Pixel_Größe   --> die /100 sind für die umrechnug von cm zu m

                return True
            
        return False
    

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





# def rotation (brauchen wir hier eigentlich nicht)

#   def linear fahrt (brauchen wir hier eigentlich nicht)



#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# *************************** Test Main (alle Nodes sollten am Ende, zentral aus einer Datei gestartet werden) ******************************

def main(args=None):
    rclpy.init(args=args)
    Camera_data_sub = Camera_data()
    
    # Timer für regelmäßigen Test
    def test_marker_erkennung():
        gefunden = Camera_data_sub.vipr.find_ArUco(114) 
        
        if gefunden:
            print(f"✓ Marker gefunden!")
            print(f"  ID: {Camera_data_sub.vipr.marker_id}")
            print(f"  Mittelpunkt: {Camera_data_sub.vipr.marker_mittelpunkt}")
            print(f"  Distanz: {Camera_data_sub.vipr.marker_distanz:.2f}m")
            print(f"  Kantenlänge: {Camera_data_sub.vipr.marker_kantenlaenge_pixel:.1f}px")
        else:
            print("✗ Kein Marker erkannt")
    
    # Alle 0.5 Sekunden testen
    timer = Camera_data_sub.create_timer(0.5, test_marker_erkennung)
    
    try:
        rclpy.spin(Camera_data_sub)
    except KeyboardInterrupt:
        pass
    finally:
        Camera_data_sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()