from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 
import rclpy
from rclpy.node import Node

from cam_VisionProcessor_logic import VisionProcessor
from datagates.msg import MarkerInfo



# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ ROS2 - SupPubNode - Zuständig für alle  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Camera_data(Node):
    def __init__(self):
        super().__init__('cam_sup_pub')
        self.bridge = CvBridge()
        self.gray_frame = None
        self.vp = VisionProcessor()

        self.subscription = self.create_subscription(
            Image,
            'pycam_tb3',
            self.frame_callback,
            10
        )
        self.pub_cam_data = self.create_publisher(
            MarkerInfo,
            'marker_info', 
            10
        )

        self.get_logger().info("SupPubNode gestartet")

        
    def frame_callback(self, msg):
        self.gray_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

#Bild da?

        if self.gray_frame is not None:
            self.get_logger().info("Graubild bei SubPub angekommen!")
        else:
            self.get_logger().info("Da knallts an der Kreuzung gewaltig")

#Visionprocessor arbeitet

        if not self.vp.find_ArUco(self.gray_frame):   #TODO: Sollten wir einen kurzen time.Sleep(0.05) einbauen, damit der Visionprozessor das bild verarbeiten und die variablen neu schreiben kann, bevor wir sie HIER eine zeile später abfragen?
            self.get_logger().info("Es wurde kein Marker gefunden, oder zu viele...")
            msg_out = MarkerInfo()
            msg_out.detected = False
            self.pub_cam_data.publish(msg_out)
            return False
        
        marker_gefunden = self.vp.marker_gefunden
        marker_id = self.vp.marker_id
        marker_distanz = self.vp.marker_distanz
        marker_winkel = self.vp.marker_winkel

        msg_out = MarkerInfo() #TODO custommessage von Yatheesh
        msg_out.detected = marker_gefunden
        msg_out.angle = marker_winkel
        msg_out.distance = marker_distanz
        msg_out.marker_id = marker_id
        
        self.pub_cam_data.publish(msg_out)

        self.get_logger().info(f'ArUco gefunden: ID: {marker_id}', f'distanz: {marker_distanz: .2f}', f'Winkel: {marker_winkel: .2f}°')




#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ***************************  Main (alle Nodes sollten am Ende, zentral aus einer Datei gestartet werden) ******************************

def main(args=None):
    rclpy.init(args=args)
    cam_sub_pub = Camera_data()
    rclpy.spin(cam_sub_pub)
    cam_sub_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 