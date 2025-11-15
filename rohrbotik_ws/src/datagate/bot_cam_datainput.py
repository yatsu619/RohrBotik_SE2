from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node


class CameraOutPut(Node):
    def __init__(self):
        super().__init__('camera_read_out') #Node-bennenung
        self.publisher = self.create_publisher(Image, 'versuchsstream', 10) # die ,10) ist die Zwischenspeicher-menge für Bilder in der Node-Warteschlange
        self.camera = cv.VideoCapture(4)                                    #Kamera-Index hier anpassen  evt mit /dev/video0
        self.bridge = CvBridge()                                            #Hier, weil sonst die Kamera bei jedem def timer_callback aufgerufen werden würde
        self.timer = self.create_timer(1/24, self.timer_callback)

    def timer_callback(self):
        succsess, frame = self.camera.read()
        if succsess: 
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("Bild wird verschickt... ")
        else:
            self.get_logger().info("Bild konnte nicht geladen werden!!")

    
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraOutPut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 