from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


class CameraOutPut(Node):

    '''
    Die CameraOutPut Node ist als eigenständige ROS2 Node für Aufnehme, umwandlung in Grauwerte und weitergabe der Kamerabilder zuständig. 
    Die als Numpy-Array ankommenden cv2 Bilder werden erst zu grauwerten berechnet und dann über die Bridge in ein passendes ROS2 Format convertiert. 
    Auf das Topic "pycam_tb3" werden pro sekunde 24 Bilder gepublished.
    '''
    def __init__(self):
        super().__init__('camera_read_out')
        self.publisher = self.create_publisher(
            Image, 
            'pycam_tb3', 
            10
        )
        
        self.camera = cv.VideoCapture(4)                                    
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/24, self.timer_callback)

    def timer_callback(self):
        succsess, frame = self.camera.read()
        if succsess: 
            frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            msg = self.bridge.cv2_to_imgmsg(frame_gray, encoding='mono8')
            self.publisher.publish(msg)
#           self.get_logger().info("Bild wird verschickt... ")
        else:
            self.get_logger().info("Bild konnte nicht aus der Kammerageladen werden!!")



#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraOutPut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 