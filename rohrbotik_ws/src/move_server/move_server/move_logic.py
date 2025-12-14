import math
import time


class PID: 
    _sollwinkel =0
    _gain = 1.1
    _sollabstand= 0.5 
    _gain_folgen =1.3

    @staticmethod
    def zur_mitte_regeln(aktueller_winkel,soll_geschwindigkeit_linear ):
        """ Regler zur mitte hin braucht den winkel um den er regelen muss sowie die voreingestellte geschwindigkeit 
         returnt die lineare Geschwindigkeit sowie die winkelgeschwingkeit  """
        if aktueller_winkel ==0 :
            return soll_geschwindigkeit_linear, 0.0
        else :
            angular_velocity = (((aktueller_winkel - PID._sollwinkel)*PID._gain)* math.pi / 180)     
            return soll_geschwindigkeit_linear, angular_velocity
        
    @staticmethod
    def abstand_und_winkel_regeln(aktueller_winkel,soll_geschwindigkeit_linear,distanz_nach_vorne_gemessen  ):
        """ Regler der den Abstand regelt aufgrund der distanz zum Marker und einer soll_geschwindigkeit dazu sort der das der bot gerade zur mitte Föhrt  """
        """ gemessener Winkel , Richtgeschwindigkeit , Abstand zum Marker (gemessen)"""

        angular_velocity = (((aktueller_winkel - PID._sollwinkel)*PID._gain)* math.pi / 180) 

        distanz_error = distanz_nach_vorne_gemessen - PID._sollabstand
        linear_velocity = distanz_error * PID._gain_folgen
        linear_velocity = max(-0.05, min(soll_geschwindigkeit_linear, linear_velocity))

        return linear_velocity, angular_velocity