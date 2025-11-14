
# ********************** Imports ********************* 

import cv2 as cv 
import cv2.aruco as aruco

import numpy as np 


# ********************* Variablen *********************  

richtung_rohr = False
mittig_im_Rohr = False
stop_it = False


# ********************** Funktionen (Aufruf in benötigten Servern) ********************* 



# ********************** Pseudo-Code********************* 

"""
def "Bilder empfangen und Bereitstellen"
-> Empfangen der Kamera bilder über ROS2 - Camera-Node
-> Wenn nötig Puffern der Bilder (wenn das programm zum auswerten länger braucht als Bilder ankommen)
-> Verarbeiten der Bilder in openCV format und umwandeln in Graustufen (falls das besser zur erkennung von ArUcomarken ist)
-> Bilder der restlichen Klasse / .py Datei breitstellen

def "and_rutate"(wird von rotate_server angefordert)

-> Bild von "Bild empfangen und Bereitstellen" einlesen
-> prüfen, ob ein Aruco marker mit ID=0 (5x5 - 200) zu sehen ist. 
-Wenn NEIN-> gib rotate_server "bool kein_marker = True" zurück (rotate_action_server dreht weiter)
-Wenn JA ->  Rechne Winkel vom mittelpunkt der Kamera zum Mittelpunkt des ArUco Markers aus. Gib diesen WInkel an den Rotation server zurück. Dieser Rotiert dann um den Winkel. 
[BONUS: Nach der ersten Fahrt, wird immer profisorisch einmal vor aufruf von and_rutate um 180 Grad gedreht]


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


(plathalter für die def "traffic" funktion
Diese Funktion soll erkennen ob ein zweiter Roboter oder anderer gegenstand im Rohr im Weg steht und sofort stehen bleiben und nen Fehler werfen. 
Alternative soll er einen anderen Roboter mit Aruco marker sehen und ihm hinterher fahren. ) 
--> Aber das kommt erst später, daher jetzt noch nicht relevant. 
"""