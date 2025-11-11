import math

class  Rotate :
    _omega = 0.63 # Drehgeschwindigkeit 
    _angle_threshold = 0.01  
    
    def __init__(self):
        self._target_angle = None

    def rotate_to_pipe(self,current_pose):
        if self._target_angle is None:
            angle_to_target = 180 * math.pi / 180 # winkel in Radianten 

        angle_diff = self.normalize_angle(angle_to_target - current_pose[2])

        if abs(angle_diff) < Rotate._angle_threshold:
            return #self._rotate_more(current_pose)
        
        return [0., Rotate._omega]
        

   # def rotate_more(self,current_pose):# achtung rekursiv nur mit abbruch bedingung aufrufen 
        new_angel=current_pose[2] + 79* math.pi / 180 
        angle_diff = self.normalize_angle(new_angel - current_pose[2])

        if abs(angle_diff) < Rotate._angle_threshold:
            return self._rotate_more(current_pose)
        
        return [0., Rotate._omega]

    def normalize_angle(angle):
        
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle