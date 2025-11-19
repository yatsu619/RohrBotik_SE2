import rclpy
from interfaces.action import rotate 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
import time
import rotate_logic
from datagate.cam_vision_manager import VisionManager


class rotate_server(Node):
    ''' KLasse um den Roboter zu drehen '''
    def __init__(self):
        ''' Beinhaltet action rotate 
            Subscribet auf Pose2D 
            Publischt auf Twist, cmd_vel 
        '''
        super().__init__('rotate_server_node')
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.current_goal_handle = None
        self.count = 0
        self.visionprocessor = VisionManager.get_instance()                             # Hierrüber lassen sich alle Funktionen des VP mit z.B. marker_gefunden = self.visionprocessor.find_ArUco(0) aufrufen

        self.control_timer = self.create_timer(0.1, self.control_step)

        self._action_server = ActionServer ( self, rotate,'rotate',         
                                                self.execute_callback,          
                                                goal_callback=self.goal_callback,        # optional: neue Ziele akzeptieren
                                                cancel_callback=self.cancel_callback,    # optional: Abbrüche akzeptieren
                                            ) 
        
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose2D,'/pose',self.pose_callback,10,)
        self.get_logger().info(' Rotate Action Server gestartet.')
      
    def goal_callback(self, goal_request):
        """Dort wird das ziel ohne bedingung auf akzeptiern gesetzt """
        self.get_logger().info(' Neues Ziel empfangen!')
        return GoalResponse.ACCEPT  

    def pose_callback(self,msg:Pose2D):
        ''' Integriert die Pose vom subscriber in die current_pose vom type Pose2D  '''
        self.current_pose.x=msg.x
        self.current_pose.y=msg.y
        self.current_pose.theta=msg.theta
        self.get_logger().info(f'Pose empfangen: x={msg.x}, y={msg.y}, theta={msg.theta}')

    def cancel_callback(self, goal_handle):
            """Dort wird das ziel ohne bedingung abgebrochen vom client  """
            return CancelResponse.ACCEPT
    """ hier die möglichkeit aufgrund der distanz zu untescheiden welcher arucomarker zum Abbruch führt """
        #if cam.distanz_to_marker  :
           # return CancelResponse.Reject 
    

    
    
    def execute_callback(self, goal_handle):
        '''Die Ausführende gewalt    '''
        self.current_goal_handle = goal_handle
        result = rotate.Result()

        return result

    def control_step(self):
        ''' Hier ist die logic integiert der Timer rufft diese in 10Hz auf hier wird geprüft ob wir das rohr erkannt haben und hier wird gedreht  '''
        if self.current_goal_handle is None:
            return

        if self.current_goal_handle.is_cancel_requested:
            self._cancel_goal('Client hat abgebrochen.')
            return

        if self.visionprocessor.find_ArUco(0):      # interface @Patrice115 
            self._finish_goal('Seerohr erkannt breche Rotation ab.')
            
            return

    
        cmd_actuell = Twist()
        if self.count == 0:
            cmd_actuell = rotate_logic.RotateCL500.rotate_to_pipe(self.current_pose)
            self.count = 1
        else:
            cmd_actuell = rotate_logic.RotateCL500.rotate_more(self.current_pose)
        self.cmd_pub.publish(cmd_actuell)


   

    def stop_motion(self):
        ''' Stop ist Stop !! noch Fragen ?'''
        stop = Twist()
        self.cmd_pub.publish(stop)


 

def main():
    rclpy.init()  
    try:
        # Node laufen lassen - blockiert hier bis Ctrl+C gedrückt wird
        rclpy.spin()
        
    except KeyboardInterrupt:
        # Wenn Benutzer Ctrl+C drückt
        rotate_server.get_logger().info('Server durch Benutzer unterbrochen...')

    except Exception as e:
        # Falls irgendein anderer Fehler auftritt
        rotate_server.get_logger().error(f'Unerwarteter Fehler: {e}')
        
    finally:
        rotate_server.get_logger().info('Server wird beendet...')
        rotate_server.stop_motion()  # Roboter stoppen
        rotate_server.destroy_node()  
        rclpy.shutdown() 

if __name__ == '__main__':
    main()





if __name__ == '__main__':
    main()
