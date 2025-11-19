import math

class  RotateCL500 :# patrice idee 'CL500'
    _omega = 0.63 # Drehgeschwindigkeit 
    _angle_threshold = 0.01  
    @staticmethod
    def __init__():
        ''' Setzt die anfangsbedingung auf Null '''
        _target_angle = None
        
    @staticmethod
    def rotate_to_pipe(current_pose):
        ''' Dreht sich um 180 Grad Gedacht um nach der Zweiten Fahrt schneller zu sein '''
        if RotateCL500._target_angle is None:
            angle_to_target = 180 * math.pi / 180 # winkel in Radianten 

        angle_diff = RotateCL500.normalize_angle(angle_to_target - current_pose[2])

        if abs(angle_diff) < RotateCL500._angle_threshold:
            return 0., 0.0
        
        return 0., RotateCL500._omega
        
    @staticmethod
    def rotate_more(current_pose):
        ''' Dreht sich um 79 Grad damit wir am anfang das rohr finden 
             79 Grad da es eine primzahl ist und so sichergestellt ist das die gleichen spots nicht doppelt aufgerufen werden 
             Außerdem werden so realativ schnell alle 4 Quartale abgedeckt --> Schnelles rohr finden + sichergestellt das das Rohr zu 100% gefunden wird  '''
        new_angel=current_pose[2] + 79* math.pi / 180 # 79 ist eine primzahl das heißt in 5 umdrehungen sind wir einmal um den kreiß und weiter sso stellen wir sicher das wir mit der kamera auf jeden fall das rohr erkenn 
        angle_diff = RotateCL500.normalize_angle(new_angel - current_pose[2])

        if abs(angle_diff) < RotateCL500._angle_threshold:
            return 0., 0.0
        ''' Sorgt dafür das nur positive Zahlen genommen werden abs() und ermöglicht so einen einfachen vergleich '''
        return 0., RotateCL500._omega
    @staticmethod
    def normalize_angle(angle):
        ''' Sorg dafür das der Winkel im bereich von - pi bis pi liegt '''

        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle