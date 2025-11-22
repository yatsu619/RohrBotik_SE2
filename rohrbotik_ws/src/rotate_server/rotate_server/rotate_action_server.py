import rclpy
from interfaces.action import RotateAc 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
import time
from rotate_server.rotate_action_server import RotateCL500
from cam_msgs.msg import MarkerInfo


class rotate_action_server(Node):
    ''' KLasse um den Roboter zu drehen '''
    def __init__(self):
        ''' Beinhaltet action rotate 
            Subscribet auf Pose2D 
            Publischt auf Twist, cmd_vel 
        '''
        

        super().__init__('rotate_server_node')
        self.current_pose = Pose2D()
        self.current_goal_handle = None
        self.count = 0
        self.marker_winkel = 0.0
        self.marker_gefunden = False
        self.rotation_active = False

        self.control_timer = self.create_timer(0.1, self.control_step)
        self.inner_counter = 0
        self.gesetzter_wki =0 

        self._action_server = ActionServer ( self, RotateAc,'rotate',         
                                                self.execute_callback,          
                                                goal_callback=self.goal_callback,        # optional: neue Ziele akzeptieren
                                                cancel_callback=self.cancel_callback,    # optional: Abbrüche akzeptieren
                                            ) 
        
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose2D,'/pose',self.pose_callback,10,)

        self.marker_info = self.create_subscription(
            MarkerInfo,
            'MarkerInfos',
            self.marker_callback,
            10
        )

        self.get_logger().info(' Rotate Action Server gestartet.')


    def marker_callback(self, msg_in):
        '''
        Empfängt Custommessage aus der Datagate und gibt diese in Instanzvariablen des Actionservers.
        > bool marker_found
        > float32 marker_distanz
        > float32 marker_winkel
        > int32 marker_id
        '''
        self.marker_winkel = msg_in.marker_winkel
        self.get_logger().info(f"Winkel empfangen: {self.marker_winkel}°")

        self.marker_gefunden = msg_in.marker_found
        self.get_logger().info(f"Marker gefunden")

      
    def goal_callback(self, goal_request):
        """Dort wird das ziel ohne bedingung auf akzeptiern gesetzt """
        self.get_logger().info(' Neues Ziel empfangen!')
        return GoalResponse.ACCEPT  

    def pose_callback(self,msg:Pose2D):
        ''' Integriert die Pose vom subscriber in die current_pose vom type Pose2D  '''
        self.current_pose = Pose2D(x=msg.x, y=msg.y, theta=msg.theta)


        #self.current_pose.x=msg.x
        #self.current_pose.y=msg.y
        #self.current_pose.theta=msg.theta
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
        self.current_goal_handle = goal_handle
        self.rotation_active = True
        #one_direction = goal_handle.request.one_direction
        self.get_logger().info(f"Starte Rotation-Goal: {self.count}")

        feedback_msg = RotateAc.Feedback()
        feedback_msg.start_winkel = self.current_pose.theta
        goal_handle.publish_feedback(feedback_msg)
        

        while self.rotation_active and rclpy.ok():
            '''rclpy.ok() ist eine Hilfsfunktion die prüft, ob das ros-System noch läuft und die Node weiter machen soll'''
            rclpy.spin_once(self, timeout_sec=0.1)

        result = RotateAc.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info('Rotation abgeschlossen')
        self.current_goal_handle = None
        return result

    def control_step(self):
        ''' Hier ist die logic integiert der Timer rufft diese in 10Hz auf hier wird geprüft ob wir das rohr erkannt haben und hier wird gedreht  '''
        self.get_logger().info('Start Controlstep ')
        if self.current_goal_handle is None or not self.rotation_active:
            return

        if self.current_goal_handle.is_cancel_requested:
            self.rotation_active = False
            self.stop_motion()
            self.get_logger().info('Cancel-Flag erkannt: Drehen gestoppt')
            #self._cancel_goal('Client hat abgebrochen.')
            return
        

       #if self.visionprocessor.find_ArUco(114):# interface @Patrice115 
        #    self.rotation_active = False
         #   self.stop_motion()
          #  self.get_logger().info('Seerohr erkannt breche Rotation ab')
           # #self._finish_goal('Seerohr erkannt breche Rotation ab.')
            #return
        
        if self.marker_gefunden:
            self.rotation_active = False
            self.stop_motion()
            self.get_logger().info('Seerohr erkannt breche Rotation ab')
            return

        
        if self.count == 0:
            linear_vel, angular_vel, gedreht_janein,self.esetzter_wki =RotateCL500.rotate_to_pipe(self.current_pose.theta, self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1 
            if gedreht_janein == True: 
                self.count == 1
                self.inner_counter = 0

        else:
            linear_vel, angular_vel, gedreht_janein,self.gesetzter_wki  = RotateCL500.rotate_more(self.current_pose.theta , self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1
            if gedreht_janein == True:
                if self.count == 10:
                    self.stop_motion
                    self.get_logger().info('2 volle Umdrehungen, Rohr nicht gefunden!')
                    return
                else:
                    self.count += 1
                self.inner_counter = 0
        

        cmd_aktuell = Twist()
        cmd_aktuell.linear.x = linear_vel
        cmd_aktuell.angular.z = angular_vel
        self.cmd_pub.publish(cmd_aktuell)

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