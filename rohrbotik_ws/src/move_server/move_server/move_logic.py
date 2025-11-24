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
         return linear,0
    else :
      rad_zur_mitte = (((0- winkel)*0.025)* math.pi / 180)

      

      return linear,rad_zur_mitte







def timer(sekunden):
    ''' nur zum test fliegt wieder raus '''
    for i in range(sekunden,-sekunden,-1):
        print(i)
        a,b=zur_mitte_regeln(i )
        print(a,'_', b)
        time.sleep(0.5)
    print("⏰ Zeit ist um!")
        
timer(90)  # 10-Sekunden-Timer

