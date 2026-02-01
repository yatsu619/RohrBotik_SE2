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
            Publisht auf Twist, cmd_vel 
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

        #self.get_logger().info(' Rotate Action Server gestartet.')


    def goal_callback(self):
        """Dort wird das ziel ohne bedingung auf akzeptiern gesetzt """ 
        """
        Wird aufgerufen wenn ein neues Goal empfangen wird.
        """
        #self.get_logger().info(' Neues Ziel empfangen!')
        return GoalResponse.ACCEPT

    def cancel_callback(self):
        """Dort wird das ziel ohne bedingung abgebrochen vom client  """
        """
        Wird aufgerufen wenn ein Goal-Abbruch angefragt wird.
        
        Stoppt die Rotation sofort und beendet execute_callback.
        
            
        CancelResponse.ACCEPT: Abbruch wird immer akzeptiert
        """
        self.get_logger().info('Rotate Abbruch angefragt')
        self.rotation_active = False
        self.stop_motion()
        self.done_event.set()   # beendet execute callback
        return CancelResponse.ACCEPT

    def marker_callback(self, msg: MarkerInfo):
        """
        Empfängt und speichert Marker-Informationen von der Kamera.
        
        Aktualisiert interne Zustandsvariablen mit aktuellen Markerdaten.
        
        msg: MarkerInfo Message mit Marker-Daten
            - marker_found: Boolean ob Marker erkannt wurde
            - marker_distanz: Distanz zum Marker in Metern
            - marker_winkel: Winkel zum Marker in Grad
            - marker_id: ID des erkannten Markers
        """
        self.marker_winkel = msg.marker_winkel
        self.marker_distanz = msg.marker_distanz
        self.marker_gefunden = msg.marker_found
        self.marker_id = msg.marker_id
        #self.get_logger().info(f'Marker empfangen: found = {self.marker_gefunden}, dist = {self.marker_distanz:.2f}m,')

    
    def execute_callback(self, goal_handle):
        """
        Führt das Rotate Goal aus.
        
        Startet die Rotation und wartet blockierend bis ein Marker gefunden
        wurde oder maximale Drehung erreicht ist. Die eigentliche Rotation
        erfolgt im control_step Timer-Callback.
        
        goal_handle: Handle des auszuführenden Goals
            
        RotateAc.Result: Ergebnis mit success=True
        """
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
        #self.get_logger().info('Rotation abgeschlossen')
        self.current_goal_handle = None
        return result

    def control_step(self):
        """
        Hier ist die logic integiert der Timer rufft diese in 10Hz auf hier wird geprüft ob wir das rohr erkannt haben und hier wird gedreht
        
        Prüft ob ein geeigneter Marker gefunden wurde und steuert die
        Rotationsbewegung. Beendet die Rotation wenn:
        - Marker 0 in ausreichender Distanz gefunden
        - Marker 69 erkannt
        - Maximale Drehungen erreicht
        """
        #Prüfen ob Rotation aktiv ist
        #self.get_logger().info('Start Controlstep ')
        if self.current_goal_handle is None or not self.rotation_active:
            return
        
        '''Prüfen ob Abbruch angefragt wurde'''
        if self.current_goal_handle.is_cancel_requested:
            self.rotation_active = False
            self.stop_motion()
            self.done_event.set()
            #self.get_logger().info('Cancel-Flag erkannt: Drehen gestoppt')
            #self._cancel_goal('Client hat abgebrochen.')
            return
        
        MINDEST_MARKER_DISTANZ = 2.0
        
        '''Prüfen ob geeignter Marker gefunden wurde'''
        if self.marker_gefunden: 
            if self.marker_id == 0 and self.marker_distanz > MINDEST_MARKER_DISTANZ:
                self.rotation_active = False
                self.stop_motion()
                self.done_event.set()
                #self.get_logger().info('Marker 0 in Reichweite, Drehung beendet!')
                return
            
            elif self.marker_id == 69:
                self.rotation_active = False
                self.stop_motion()
                self.done_event.set()
                #self.get_logger().info('Marker 69 erkannt, Drehung beendet!')
                return
        '''Rotationslogik: Erste Drehung'''
        if self.count == 0:
            linear_vel, angular_vel, gedreht_janein,self.gesetzter_wki =RotateCL500.rotate_to_pipe(self.marker_winkel, self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1 
            if gedreht_janein == True: 
                self.count = 1
                self.inner_counter = 0

        #Zweite Drehung: je 79°
        else:
            linear_vel, angular_vel, gedreht_janein,self.gesetzter_wki  = RotateCL500.rotate_more(self.marker_winkel , self.inner_counter,self.gesetzter_wki )
            self.inner_counter += 1
            if gedreht_janein == True:
                '''Nach 10 Drehungen abbrechen'''
                if self.count == 10:
                    self.rotation_active = False
                    self.stop_motion()
                    self.done_event.set()
                    #self.get_logger().info('2 volle Umdrehungen, Rohr nicht gefunden!')
                    return
                else:
                    self.count += 1
                self.inner_counter = 0

        # Bewegung ausführen
        cmd_aktuell = Twist()
        cmd_aktuell.linear.x = float(linear_vel)
        cmd_aktuell.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd_aktuell)

        # Feedback senden
        feedback_msg = RotateAc.Feedback()
        self.current_goal_handle.publish_feedback(feedback_msg)

    def stop_motion(self):
        ''' Stoppt den Roboter sofort '''
        stop = Twist()
        self.cmd_pub.publish(stop)
        #self.get_logger().info('Roboter gestoppt')

def main(args=None):
    ''' 
    Hauptfunktion zum Starten des Rotate Action Servers.
    Initialisiert ROS2, erstellt die Node und startet den MultiThreadedExecutor.
    Behandelt Keyboard Interrupts und Exceptions für sauberen Shutdown
    '''
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