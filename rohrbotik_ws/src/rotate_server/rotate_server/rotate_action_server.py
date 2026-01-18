from .rotate_logic import RotateCL500
import rclpy
from interfaces.action import RotateAc 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from cam_msgs.msg import MarkerInfo
import threading


class RotateActionServer(Node):
    ''' KLasse um den Roboter zu drehen '''
    def __init__(self):
        ''' Beinhaltet action rotate 
            Subscribet auf Pose2D 
            Publischt auf Twist, cmd_vel 
        '''
        

        super().__init__('rotate_server_node')
        self.callback_group = ReentrantCallbackGroup()
        self.done_event = threading.Event()
        
        self.current_goal_handle = None
        self.count = 0
        self.marker_distanz = 0.0
        self.marker_winkel = 0.0
        self.marker_id = 0
        self.marker_gefunden = False
        self.rotation_active = False

        self.control_timer = self.create_timer(0.1,
                                               self.control_step,
                                               callback_group=self.callback_group)
        
        self.inner_counter = 0
        self.gesetzter_wki =0 

        self._action_server = ActionServer (self,
                                            RotateAc,
                                            'rotate',         
                                            self.execute_callback,          
                                            goal_callback=self.goal_callback,        # optional: neue Ziele akzeptieren
                                            cancel_callback=self.cancel_callback,    # optional: Abbrüche akzeptieren
                                            callback_group=self.callback_group) 
        
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        

        self.marker_info = self.create_subscription(MarkerInfo,
                                                    'MarkerInfos',
                                                    self.marker_callback,
                                                    10,
                                                    callback_group=self.callback_group)

        self.get_logger().info(' Rotate Action Server gestartet.')


    def goal_callback(self, goal_request):
        """Dort wird das ziel ohne bedingung auf akzeptiern gesetzt """
        self.get_logger().info(' Neues Ziel empfangen!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Dort wird das ziel ohne bedingung abgebrochen vom client  """
        self.get_logger().info('Rotate Abbruch angefragt')
        self.rotation_active = False
        self.stop_motion()
        self.done_event.set()   # beendet execute callback
        return CancelResponse.ACCEPT

    def marker_callback(self, msg: MarkerInfo):
        '''
        Empfängt Custommessage aus der Datagate und gibt diese in Instanzvariablen des Actionservers.
        > bool marker_found
        > float32 marker_distanz
        > float32 marker_winkel
        > int32 marker_id
        '''
        self.marker_winkel = msg.marker_winkel
        self.marker_distanz = msg.marker_distanz
        self.marker_gefunden = msg.marker_found
        self.marker_id = msg.marker_id
        self.get_logger().info(f'Marker empfangen: found = {self.marker_gefunden}, dist = {self.marker_distanz:.2f}m,')

    
    def execute_callback(self, goal_handle):
        '''Die Ausführende gewalt    '''
        self.current_goal_handle = goal_handle
        self.rotation_active = True
        self.done_event.clear()

        self.count = 0
        self.inner_counter = 0
        self.gesetzter_wki = 0.0

        self.done_event.wait()

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
            self.done_event.set()
            self.get_logger().info('Cancel-Flag erkannt: Drehen gestoppt')
            #self._cancel_goal('Client hat abgebrochen.')
            return
        
        MINDEST_MARKER_DISTANZ = 2.0
        
        if self.marker_gefunden: 
            if self.marker_id == 0 and self.marker_distanz > MINDEST_MARKER_DISTANZ:
                self.rotation_active = False
                self.stop_motion()
                self.done_event.set()
                self.get_logger().info('Marker 0 in Reichweite, Drehung beendet!')
                return
            
            elif self.marker_id == 69:
                self.rotation_active = False
                self.stop_motion()
                self.done_event.set()
                self.get_logger().info('Marker 69 erkannt, Drehung beendet!')
                return

        if self.count == 0:
            linear_vel, angular_vel, gedreht_janein,self.gesetzter_wki =RotateCL500.rotate_to_pipe(self.marker_winkel, self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1 
            if gedreht_janein == True: 
                self.count = 1
                self.inner_counter = 0

        else:
            linear_vel, angular_vel, gedreht_janein,self.gesetzter_wki  = RotateCL500.rotate_more(self.marker_winkel , self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1
            if gedreht_janein == True:
                if self.count == 10:
                    self.rotation_active = False
                    self.stop_motion()
                    self.done_event.set()
                    self.get_logger().info('2 volle Umdrehungen, Rohr nicht gefunden!')
                    return
                else:
                    self.count += 1
                self.inner_counter = 0
        

        cmd_aktuell = Twist()
        cmd_aktuell.linear.x = float(linear_vel)
        cmd_aktuell.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd_aktuell)

        feedback_msg = RotateAc.Feedback()
        #feedback_msg.start_winkel = self.current_pose.theta
        self.current_goal_handle.publish_feedback(feedback_msg)

    def stop_motion(self):
        ''' Stop ist Stop !! noch Fragen ?'''
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.get_logger().info('Roboter gestoppt')

def main(args=None):
    rclpy.init(args=args)
    node = RotateActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Node laufen lassen - blockiert hier bis Ctrl+C gedrückt wird
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Wenn Benutzer Ctrl+C drückt
        node.get_logger().info('Server unterbrochen')
        node.stop_motion()
    except Exception as e:
        # Falls irgendein anderer Fehler auftritt
        node.get_logger().error(f'Unerwarteter Fehler: {e}')
    finally:
        node.get_logger().info('Server wird beendet...')
        #node.stop_motion()  # Roboter stoppen
        node.destroy_node()  
        rclpy.shutdown() 


if __name__ == '__main__':
    main()