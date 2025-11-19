from cam_data_sub_Node import Camera_data
from cam_vision_manager import VisionManager
import numpy as np
import cv2.aruco as aruco

import time
import cv2 as cv


ViMa = VisionManager.get_instance()

def main():
    while True:
        aruco_ja = ViMa.find_ArUco(114)
        print(aruco_ja)
        
        bild = ViMa.showImg()
        print(type(bild))
        #cv.imshow("aktuelles Bild", bild)
        #cv.waitKey(0)
        #cv.destroyAllWindows
        
        time.sleep(5)

if __name__ == '__main__': 
    main()

