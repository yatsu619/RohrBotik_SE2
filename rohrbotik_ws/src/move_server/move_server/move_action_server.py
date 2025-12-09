from .move_logic import PID
import rclpy
from interfaces.action import MoveAc
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cam_msgs.msg import MarkerInfo
import threading

class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_server_node')
        '''für Multi-Threading: control-step und execute callback können gleichzeitig laufen'''
        self.callback_group = ReentrantCallbackGroup()  
        self.done_event = threading.Event()     #event-getriggert, ersetzt die while schleife im execute callback
        self.current_goal_handle = None
        self.move_active = False
        self.target_vel = 0.0
        self.marker_found = False
        self.marker_puffer=0 
        self.marker_distanz = 0.0
        self.marker_winkel = 0.0
        self.marker_id = 0             

        self.control_timer = self.create_timer(0.1, 
                                               self.control_step,
                                               callback_group = self.callback_group)
        
        self._action_server = ActionServer(self,
                                            MoveAc,
                                            'move', 
                                            self.execute_callback,
                                            goal_callback = self.goal_callback,
                                            cancel_callback = self.cancel_callback,
                                            callback_group = self.callback_group)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.marker_sub = self.create_subscription(MarkerInfo,
                                                    'MarkerInfos',              #topic wurde von marker_info zu MarkerInfos geändert
                                                    self.marker_callback,
                                                    10,
                                                    callback_group = self.callback_group)
        
        self.get_logger().info('Move Action Server wurde gestartet')


    def goal_callback(self, goal_request):
        self.get_logger().info(f'Neues Move-Ziel: {goal_request.target_vel}')
        return GoalResponse.ACCEPT
            
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Move Abbruch angefragt')
        self.move_active = False
        self.stop_motion()
        self.done_event.set()   #beendet execute callback
        return CancelResponse.ACCEPT
            
    def marker_callback(self, msg: MarkerInfo):
        self.marker_found = msg.marker_found
        self.marker_distanz = msg.marker_distanz
        self.marker_winkel = msg.marker_winkel
        self.marker_id = msg.marker_id
        self.get_logger().info(f'Marker empfangen: found = {self.marker_found}, dist = {self.marker_distanz:.2f}m')
        if self.marker_found== False:
             self.marker_puffer +=1
        if self.marker_found== True:
             self.marker_puffer= 0 
    


    def execute_callback(self, goal_handle):            #goal_handle hat das goal und die Methoden für feedback und result
        self.current_goal_handle = goal_handle          #speichert goal_handle in eine Instanzvariable 
        self.move_active = True
        self.done_event.clear()  #setzt das event zurück für den neuen durchlauf

        self.target_vel = goal_handle.request.target_vel
        self.target_vel = max(0.0, min(0.2, self.target_vel))

        self.get_logger().info(f'Linearfahrt starten mit {self.target_vel} m/s')

        self.done_event.wait()  #warten auf event, welches im control-step gesetzt wird

        result = MoveAc.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info('Move abgeschlossen')
        self.current_goal_handle = None         #löscht das aktuelle goal_handle damit control_step weiß, dass keine action läuft
        return result
    
    def control_step(self):
        '''Timer-Callback, welches alle 0,1s aufgerufen wird (10Hz)'''

        if not self.move_active or self.current_goal_handle is None:       #Wenn der Bot nicht fährt, soll er auch nichts tun
              return
         
        if self.current_goal_handle.is_cancel_requested:            #Wenn Abbruch angefragt wurde
              self.move_active = False
              self.stop_motion()
              self.done_event.set()
              self.get_logger().info('Move abgebrochen')
              return
        
        
         
        MARKER_STOPP_DISTANZ = 0.45  
        #if self.marker_found:
            #self.get_logger().info(f'Check: marker_found = {self.marker_found}, dist = {self.marker_distanz:.2f}m, stopp_bei = {MARKER_STOPP_DISTANZ}m')

        if self.marker_found and self.marker_distanz < MARKER_STOPP_DISTANZ:
              self.move_active = False
              self.stop_motion()
              self.get_logger().info(f'Marker erreicht')
              self.done_event.set() #Event setzen, um execute callback zu beenden
              return
        if self.marker_found== True :
            self.get_logger().info(f'VOR PID: self.marker_winkel = {self.marker_winkel:.2f}°')          #Test log
            '''Aufruf des Reglers'''
            linear_vel, angular_vel = PID.zur_mitte_regeln(self.marker_winkel, self.target_vel)     
            self.get_logger().info(f'PID: winkel = {self.marker_winkel:.2f}°, linear = {linear_vel:.3f}, angular = {angular_vel:.3f}') 
        elif self.marker_found== False and self.marker_puffer >=12 :
            """ Wir prüfen ob wir den marker sehen und wenn nicht warten wir bis wir ihn 12 mal nicht sehen bevor wir stehen bleiben bei 24Hz entspricht 12 einer halben sekunde """
            self.stop_motion()
            self.get_logger().info(f'Keine Sicht,Keine Fahrt ')

        cmd = Twist()
        '''Geschwindigkeit publishen'''
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)      #konvertieren damit z immer float ist
        self.cmd_pub.publish(cmd)               #sendet twist_message auf das topic cmd_vel

        feedback = MoveAc.Feedback()
        feedback.aktuelle_vel = linear_vel
        feedback.marker_erkannt = self.marker_found
        self.current_goal_handle.publish_feedback(feedback)

    def stop_motion(self):
        '''Roboter sofort anhalten'''
        stop = Twist()
        self.cmd_pub.publish(stop)                      #stop an cmd_vel senden
        self.get_logger().info('Roboter gestoppt')

def main(args = None):
    rclpy.init(args = args)
    node = MoveActionServer()               #erstellt instanz des move action servers ->init wird aufgerufen und erstellt das oben angegebene
    executor = MultiThreadedExecutor()      #executor: entscheidet wann welcher callback ausgeführt wird mit mehreren threads
    executor.add_node(node)                 #fügt den move action server dem executor hinzu

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Server unterbrochen')
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
