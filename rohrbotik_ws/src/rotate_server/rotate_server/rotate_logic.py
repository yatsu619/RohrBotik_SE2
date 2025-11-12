import math

class  RotateCL500 :# patrice idee 'CL500'
    _omega = 0.63 # Drehgeschwindigkeit 
    _angle_threshold = 0.01  
    
    def __init__(self):
        self._target_angle = None

    def rotate_to_pipe(self,current_pose):
        if self._target_angle is None:
            angle_to_target = 180 * math.pi / 180 # winkel in Radianten 

        angle_diff = self.normalize_angle(angle_to_target - current_pose[2])

        if abs(angle_diff) < RotateCL500._angle_threshold:
            return True
        
        return [0., RotateCL500._omega]
        

    def rotate_more(self,current_pose):
        new_angel=current_pose[2] + 79* math.pi / 180 # 79 ist eine primzahl das heißt in 5 umdrehungen sind wir einmal um den kreiß und weiter sso stellen wir sicher das wir mit der kamera auf jeden fall das rohr erkenn 
        angle_diff = self.normalize_angle(new_angel - current_pose[2])

        if abs(angle_diff) < RotateCL500._angle_threshold:
            return True
        
        return [0., RotateCL500._omega]

    def normalize_angle(angle):
        
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle