import math
import time


class PID: 
    """   Einfache Steuerung für Roboterbewegungen mit Winkel- und Abstandskorrektur.
  Zielwinkel für die Regelung (0°),Verstärkung für Winkelkorrektur (1.1),Verstärkung für Abstandsregelung (1.3). """
    _sollwinkel =0
    _gain = 1.1
    _gain_folgen =1.3

    @staticmethod
    def zur_mitte_regeln(aktueller_winkel,soll_geschwindigkeit_linear ):
        """
        Berechnet die Steuerung, um den Roboter auf den Sollwinkel (Mitte) auszurichten.
        Args:aktueller_winkel (float): Aktuell gemessener Winkel [Grad].
            soll_geschwindigkeit_linear (float): Vorgabe für die lineare Geschwindigkeit.
        Returns:tuple: (lineare Geschwindigkeit, Winkelgeschwindigkeit in rad/s)
        Beschreibung:
            Die Methode berechnet die Winkelgeschwindigkeit proportional zur Abweichung
            vom Sollwinkel. Die lineare Geschwindigkeit bleibt konstant.
        """
        angular_velocity = (((aktueller_winkel - PID._sollwinkel)*PID._gain)* math.pi / 180)     
        return soll_geschwindigkeit_linear, angular_velocity
        
    @staticmethod
    def abstand_und_winkel_regeln(aktueller_winkel,soll_geschwindigkeit_linear,distanz_nach_vorne_gemessen, soll_abstand):
        """
        Berechnet lineare und Winkelgeschwindigkeit basierend auf Abstand und Winkel.
        Args:aktueller_winkel (float): Aktuell gemessener Winkel [Grad].
            soll_geschwindigkeit_linear (float): Vorgabe für die lineare Geschwindigkeit.
            distanz_nach_vorne_gemessen (float): Gemessener Abstand zum Marker/Objekt.
            soll_abstand (float): Gewünschter Soll-Abstand zum Objekt.
        Returns: tuple: (lineare Geschwindigkeit, Winkelgeschwindigkeit in rad/s)
        Beschreibung:
            Die Methode passt die lineare Geschwindigkeit proportional zur Abweichung 
            vom Soll-Abstand an und korrigiert gleichzeitig den Winkel zur Sollrichtung.
            Die lineare Geschwindigkeit wird auf [-0.05, soll_geschwindigkeit_linear] begrenzt.
        """

        angular_velocity = (((aktueller_winkel - PID._sollwinkel)*PID._gain)* math.pi / 180) 

        linear_velocity = max(-0.05, min(soll_geschwindigkeit_linear, (distanz_nach_vorne_gemessen - soll_abstand)* PID._gain_folgen))

        return linear_velocity, angular_velocity
