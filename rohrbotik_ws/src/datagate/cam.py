
# ********************** Imports ********************* 

import cv2 as cv 
import cv2.aruco as aruco

import numpy as np 


# ********************* Variablen *********************  

richtung_rohr = False
mittig_im_Rohr = False
stop_it = False


# ********************** Funktionen (Aufruf in benötigten Servern) ********************* 



# ********************** Psydo-Code********************* 

"""
def "Bilder empfangen und Bereitstellen"
-> Empfangen der Kamera bilder über ROS2 - Camera-Node
-> Wenn nötig Puffern der Bilder (wenn das programm zum auswerten länger braucht als Bilder ankommen)
-> Verarbeiten der Bilder in openCV format und umwandeln in Graustufen (falls das besser zur erkennung von ArUcomarken ist)
-> Bilder der restlichen Klasse / .py Datei breitstellen

def "and_rutate"(wird von rotate_server angefordert)
[Zusatzinfo: im rotate_server wird über die Odometrie eine Drehwegung  im Urzeigersinn eingeleitet. Der Roboter dreht sich auf der stelle und macht alle paar grad ein "Bild". 
Wenn entweder ein ArUcomarker im Bild zu sehen ist, wird entweder langsamer gedreht oder abgeschätzt wie weit man sich ncoh drehen müsste bis der zweite zu sehen ist. Dann wird ausgerichtet, so das die Kamera mitte genau mittig von den beiden Markern steht]
-> Bild von "Bild empfangen und Bereitstellen" einlesen
-> linker ArUco Marker sollte erkannt werden. Dann leichtere Drehung oder berechnete Drehung (Wenn möglich, berechnen des mittelpunktes der Kamera auf x-Achse, waagerecht, zum linken aruco und dann die entsprechende fehlende gradzahl drehen lassen)
--> Grundlegend dreht der rotate-server und frägt dann immer wieder dises Funktion hier nach der Bildauswertung des aktuellen Bildes ab. 
-> Wenn die Schilder perfekt in der Mitte platziert sind und die Abfrage kommt vom rotate_server, dann soll die Info richtung_rohr = True als bool abgegeben werden. 

[Zusatzinfo:
rotate_server - dreht paar grad und frägt, ob wir marker sehen;
def "and_rutate" - sieht oder sieht weder linken noch rechten Marker;
rotate_server - dreht wieder paar grad und frägt;
def "and_rutate" - sieht linken Marker, berechnet wie weit dieser vom Mittelpunkt der kamera waagerecht weg ist und gibt dann an rotate_server die ncoh zu drehende gradzahl;
rotate_server -  dreht und frägt wieder ab;
def "and_rutate" - prüft und bei zu großer abweichung zur mitte, wird erneut berechnet und gradzahl an server geschickt, alternative passt die ausrichtung und wir geben den bool als True zurück
rotate_server - schließt action erfolgreich ab!
]

def "linear_regelung"(wird von move_server angefordert)
-> Bild von "Bild empfangen und Bereitstellen" einlesen.
-> ArUco-Marker am Ende des Tunnels (platziert am Ende der Strecke) erkennen und anvisieren. 
-> Der Marker soll dann immer in der Mitte des bildes sein. Wenn nicht, muss die Abweichung berechnet werden und per variable gespeichert werden. 
-> der Move_server frägt das alle paar cm ab. Wenn eine zu große Abweichung entsteht muss Move_server per ODOM um den berechneten winkel (abweichung) gegenlenken. 

def "stop_it" (wird von move_server angefordert)
-> Bild von "Bild empfangen und Bereitstellen" einlesen.
-> ArUco-Marker am Ende des Tunnels (platziert am Ende der Strecke) erkennen und anvisieren.
-> Distanz zum End-Marker berechnen. 
-> ist die Distanz kleiner als eine Vordefinierte Strecke (kurz vor dem Schild ist auch der "Stopp-Puntk"), dann soll die Info ebenso in einer Variable gespeichert werden. 
-> move_server frägt ob er stehen bleiben soll über diesen Bool "stop_it"

(plathalter für die def "traffic" funktion
Diese Funktion soll erkennen ob ein zweiter Roboter oder anderer gegenstand im Rohr im Weg steht und sofort stehen bleiben und nen Fehler werfen. 
Alternative soll er einen anderen Roboter mit Aruco marker sehen und ihm hinterher fahren. ) 
--> Aber das kommt erst später, daher jetzt noch nicht relevant. 
"""