import math
import time


class PID: 
    _sollwinkel =0
    _gain = 1.1

    @staticmethod
    def zur_mitte_regeln(aktueller_winkel,soll_geschwindigkeit_linear ):
        """ Regler zur mitte hin braucht den winkel um den er regelen muss sowie die voreingestellte geschwindigkeit 
         returnt die lineare Geschwindigkeit sowie die winkelgeschwingkeit  """
        if aktueller_winkel ==0 :
            return soll_geschwindigkeit_linear, 0.0
        else :
            angular_velocity = (((aktueller_winkel - PID._sollwinkel)*PID._gain)* math.pi / 180)     
            return soll_geschwindigkeit_linear, angular_velocity