import math
import time


class PID:
    _liner_langsam = 0.080 
    _linear= 0.090 
    _angle_threshold = 0.01

    @staticmethod
    def zur_mitte_regeln(winkel,linear ):
        """ Regler zur mitte hin braucht den winkel um den er regelen muss """
        if winkel ==0 :
            return linear, 0.0
        else :
            rad_zur_mitte = (((0- winkel)*1.1)* math.pi / 180) * -1  
        #                     ^^^^                              ^^^^
        #                   Invertiert                       Invertiert

    
            return linear, rad_zur_mitte