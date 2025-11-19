import rclpy
from interfaces.action import Rotate 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
import time
import rotate_logic
from datagate.cam_vision_manager import VisionManager


class rotate_action_server(Node):
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
        self.rotation_active = False

        self.control_timer = self.create_timer(0.1, self.control_step)

        self._action_server = ActionServer ( self, Rotate,'rotate',         
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
            self.get_logger().info('Abbruch angefragt')
            if self.current_goal_handle:
                self.rotation_active = False
            return CancelResponse.ACCEPT
    """ hier die möglichkeit aufgrund der distanz zu untescheiden welcher arucomarker zum Abbruch führt """
        #if cam.distanz_to_marker  :
           # return CancelResponse.Reject 
    
    
    def execute_callback(self, goal_handle):
        '''Die Ausführende gewalt    '''
        self.get_logger().info(f"Starte Rotation-Goal: {goal_handle.request}")
        self.current_goal_handle = goal_handle
        self.rotation_active = True

        while self.rotation_active and rclpy.ok():
            time.sleep(0.1)

        result = Rotate.Result()
        goal_handle.succeed()
        self.get_logger().info('Rotation abgeschlossen')
        self.current_goal_handle = None
        return result

    def control_step(self):
        ''' Hier ist die logic integiert der Timer rufft diese in 10Hz auf hier wird geprüft ob wir das rohr erkannt haben und hier wird gedreht  '''
        if self.current_goal_handle is None or not self.rotation_active:
            return

        if self.current_goal_handle.is_cancel_requested:
            self.rotation_active = False
            self.stop_motion()
            self.get_logger().info('Cancel-Flag erkannt: Drehen gestoppt')
            #self._cancel_goal('Client hat abgebrochen.')
            return

        if self.visionprocessor.find_ArUco(0):# interface @Patrice115 
            self.rotation_active = False
            self.stop_motion()
            self.get_logger().info('Seerohr erkannt breche Rotation ab')
            #self._finish_goal('Seerohr erkannt breche Rotation ab.')
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


 

def main(args=None):
    rclpy.init(args=args)
    node = rotate_action_server()
    try:
        # Node laufen lassen - blockiert hier bis Ctrl+C gedrückt wird
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Wenn Benutzer Ctrl+C drückt
        rotate_action_server.get_logger().info('Server durch Benutzer unterbrochen...')

    except Exception as e:
        # Falls irgendein anderer Fehler auftritt
        rotate_action_server.get_logger().error(f'Unerwarteter Fehler: {e}')
        
    finally:
        rotate_action_server.get_logger().info('Server wird beendet...')
        rotate_action_server.stop_motion()  # Roboter stoppen
        rotate_action_server.destroy_node()  
        rclpy.shutdown() 

if __name__ == '__main__':
    main()