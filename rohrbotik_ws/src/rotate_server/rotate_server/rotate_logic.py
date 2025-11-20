import math

class  RotateCL500 :# patrice idee 'CL500'
    _omega = 0.63 # Drehgeschwindigkeit 
    _angle_threshold = 0.01  

   

    @staticmethod
    def rotate_to_pipe(current_pose,zähler,angle_to_target):
        ''' Dreht sich um 180 Grad Gedacht um nach der Zweiten Fahrt schneller zu sein '''
        if zähler==0:
            angle_to_target=current_pose

            angle_to_target += 180 * math.pi / 180 # winkel in Radianten 

        
        
        
        ziel_erreicht=False
        angle_diff = RotateCL500.normalize_angle(angle_to_target - current_pose)
        if abs(angle_diff) < RotateCL500._angle_threshold:
            ziel_erreicht=True
            return 0., 0.0, ziel_erreicht
        
        return 0., RotateCL500._omega,ziel_erreicht,angle_to_target
        
    @staticmethod
    def rotate_more(current_pose,zähler,new_angel ):
        ''' Dreht sich um 79 Grad damit wir am anfang das rohr finden 
             79 Grad da es eine primzahl ist und so sichergestellt ist das die gleichen spots nicht doppelt aufgerufen werden 
             Außerdem werden so realativ schnell alle 4 Quartale abgedeckt --> Schnelles rohr finden + sichergestellt das das Rohr zu 100% gefunden wird  '''
        if zähler==0 :
            new_angel=current_pose + 79* math.pi / 180 # 79 ist eine primzahl das heißt in 5 umdrehungen sind wir einmal um den kreiß und weiter sso stellen wir sicher das wir mit der kamera auf jeden fall das rohr erkenn 
            #new_angel=RotateCL500.normalize_angle(new_angel)
        
            
        ziel_erreicht=False
        angle_diff = RotateCL500.normalize_angle(new_angel - current_pose)
        if abs(angle_diff) < RotateCL500._angle_threshold:
            ziel_erreicht=True
            return 0., 0.0, ziel_erreicht
        ''' Sorgt dafür das nur positive Zahlen genommen werden abs() und ermöglicht so einen einfachen vergleich '''
        return 0., RotateCL500._omega,ziel_erreicht,new_angel 
    
    
    @staticmethod
    def rotate_to_marker(current_pose, marker_winkel_relativ):
        ''' Neu: Dreht direkt zum erkannten Marker
            marker_winkel_relativ: Relativer Winkel vom Roboter zum Marker (aus Kamera)'''
        #Berechne Zielwinkel: aktuelle Pose + relativer Markerwinkel
        target_angle = current_pose + (marker_winkel_relativ * math.pi / 180)
        target_angle = RotateCL500.normalize_angle(target_angle)

        #Berechne Winkeldifferenz
        angle_diff = RotateCL500.normalize_angle(target_angle - current_pose)

        if abs(angle_diff) < RotateCL500._angle_threshold:
            return 0., 0.0, True  #Ziel erreicht
        
        angular_vel = RotateCL500._omega if angle_diff > 0 else -RotateCL500._omega
        return 0., angular_vel, False



    @staticmethod
    def normalize_angle(angle):
        ''' Sorg dafür das der Winkel im bereich von - pi bis pi liegt '''

        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle