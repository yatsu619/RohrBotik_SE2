from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import threading
import rclpy
from interfaces.action import HandlerAc, RotateAc, MoveAc
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node

'''
Handler Server koordiniert Rotate und Move Server über Future-basierte asynchrone Kommunikation.
Aufbau ähnlich wie Rotate und Move Server (gleiche Threading-Logik und
Event-Synchronisation). Hauptunterschied: Kein eigener Timer, Steuerung erfolgt
über Callback-Kette zwischen Rotate und Move.
'''

class HandlerActionServer(Node):
    '''
    Zentrale Koordinationskomponente der Navigation.

    Unterschiede zu Rotate und Move Server:
        - Fungiert gleichzeitig als Action Server (empfängt Handler Goal)
          und als Action Client (sendet Goals an Rotate und Move)
        - Keine Timer-basierte Kontrollfunktion, Ablauf wird über Future-Callbacks gesteuert
        - Callback-Kette: Rotate fertig → Move starten → Move fertig → Rotate starten
        - Wartet beim Start auf Verfügbarkeit von Rotate und Move Server
    '''
    def __init__(self):
        super().__init__('handler_server_node')
        #Instanzvariablen
        self.callback_group = ReentrantCallbackGroup()
        self.done_event = threading.Event()
        self.current_goal_handle = None
        self.handler_active = False     #Flag, damit bei Abbruch keine neue Action
        #self.target_vel = 0.0
        #self.state = "IDLE"
        self.current_cycle = 0
        self.mission_success = True
        self.end_reason = ""

        self._action_server = ActionServer(self,
                                           HandlerAc,
                                           'handler',
                                           self.execute_callback,
                                           goal_callback = self.goal_callback,
                                           cancel_callback = self.cancel_callback,
                                           callback_group=self.callback_group)
        
        self.move_client = ActionClient(self,
                                        MoveAc,
                                        'move',
                                        callback_group=self.callback_group)
        
        self.rotate_client = ActionClient(self,
                                          RotateAc,
                                          'rotate',
                                          callback_group=self.callback_group)
        
        self.get_logger().info('Handler Action Server gestartet')


        if not self.rotate_client.wait_for_server(timeout_sec = 10.0):
            self.get_logger().warn('Rotate Server nicht gefunden. Handler wird beendet')
            return
        else:
            self.get_logger().info('Rotate Server gefunden')
        
        if not self.move_client.wait_for_server(timeout_sec = 10.0):
            self.get_logger().warn('Move Server nicht gefunden. Handler wird beendet')
            return
        else:
            self.get_logger().info('Move Server gefunden')
                
    def goal_callback(self):
        self.get_logger().info(f'Neues Handler-Goal empfangen')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self):
        self.get_logger().info('Handler Abbruch angefragt')
        self.handler_active = False
        self.mission_success = True # Abbruch ist kein Fehler
        self.end_reason = "Abgebrochen durch Client"
        self.done_event.set()
        return CancelResponse.ACCEPT
        
    def execute_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        self.handler_active = True
        self.done_event.clear()

        self.current_cycle = 0
        self.mission_success = True
        self.end_reason = ""

        self.get_logger().info(f'Handler Mission starten')
                               

        self.start_rotate()

        self.done_event.wait()

        result = HandlerAc.Result()
        result.success = self.mission_success
        result.completed_cycles = self.current_cycle
        result.end_reason = self.end_reason

        if self.mission_success:
            goal_handle.succeed()
            self.get_logger().info(f'Erfolgreich beendet: {self.current_cycle} Zyklen, Grund: {self.end_reason}')
        else:
            goal_handle.abort()
            self.get_logger().info(f'Fehlgeschlagen nach {self.current_cycle} Zyklen, Grund: {self.end_reason}')
        self.current_goal_handle = None
        return result
    
    def start_rotate(self):
        '''
        Sendet ein Goal an den Rotate Server.

        Nutzt send_goal_async um das Goal asynchron zu senden.
        Registriert rotate_goal_response_callback auf dem Future,
        dieser wird automatisch aufgerufen wenn das Goal akzeptiert wird.
        '''
        if not self.handler_active:
            return
        
        self.get_logger().info('Starte ROTATE')
        self.send_feedback("ROTATE")
        rotate_goal = RotateAc.Goal()

        send_goal_future = self.rotate_client.send_goal_async(rotate_goal)
        
        send_goal_future.add_done_callback(self.rotate_goal_response_callback)  #wird in eine interne Liste hinzugefügt, wird aufgerufen wenn start_rotate fertig ist

    def rotate_goal_response_callback(self, future):
        '''
        Wird aufgerufen wenn Rotate Server auf das Goal reagiert.

        Prüft ob das Goal akzeptiert wurde. Bei Akzeptanz wird
        get_result_async aufgerufen und rotate_result_callback registriert,
        dieser wird aufgerufen wenn die Rotation abgeschlossen ist.

        future: Future von send_goal_async mit dem GoalHandle als Ergebnis
        '''
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Rotate Goal abgelehnt')
            self.mission_success = False
            self.end_reason = "ROTATE_GOAL_REJECTED"
            self.done_event.set()
            return
        
        self.get_logger().info('Rotate Goal akzeptiert')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.rotate_result_callback)    


    def rotate_result_callback(self, future):
        '''
        Wird aufgerufen wenn Rotation abgeschlossen ist.

        Bei Erfolg wird start_move() aufgerufen, die Callback-Kette geht weiter.
        Bei Fehler wird die Mission beendet.

        future: Future von get_result_async mit dem Rotate Ergebnis
        '''
        result = future.result().result

        if not self.handler_active:
            return
        
        if result.success:
            self.get_logger().info('Rotate erfolgreich - Rohr gefunden')
            #self.state = "MOVING"
            self.start_move()
        else:
            self.get_logger().error('Rotate fehlgeschlagen - Rohr nicht gefunden')
            self.mission_success = False
            self.end_reason = "ROTATE_FAILED__NO_PIPE_FOUND"
            self.done_event.set()

    def start_move(self):
        '''
        Sendet ein Goal an den Move Server.

        Gleiche Struktur wie start_rotate, sendet Goal asynchron und
        registriert move_goal_response_callback auf dem Future.
        '''
        if not self.handler_active:
            return
        
        self.get_logger().info(f'[Zyklus {self.current_cycle +1}] Starte MOVE')
        self.send_feedback("MOVE")

        move_goal = MoveAc.Goal()
        #move_goal.target_vel = self.target_vel

        send_goal_future = self.move_client.send_goal_async(move_goal)
        
        send_goal_future.add_done_callback(self.move_goal_response_callback)

    def move_goal_response_callback(self, future):
        '''
        Gleiche Struktur wie rotate_goal_response_callback.

        Prüft Akzeptanz und registriert move_result_callback für das Ergebnis.

        future: Future von send_goal_async mit dem GoalHandle als Ergebnis
        '''
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Move Goal abgelehnt')
            self.mission_success = False
            self.end_reason = "MOVE_GOAL_REJECTED"
            self.done_event.set()
            return
        
        self.get_logger().info('Move Goal akzeptiert')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.move_result_callback)
    
    
    def move_result_callback(self, future):
        '''
        Wird aufgerufen wenn Fahrt abgeschlossen ist.

        Bei Erfolg wird der Zyklus-Zähler erhöht und start_rotate() aufgerufen –
        damit schließt sich die Callback-Kette wieder.
        Bei Fehler wird die Mission beendet.

        future: Future von get_result_async mit dem Move Ergebnis
        '''
        result = future.result().result

        if not self.handler_active:
            return
        
        if result.success:
            self.get_logger().info('Move erfolgreich abgeschlossen - Wendeplattform erreicht')
            self.current_cycle += 1
            #self.state = "ROTATING"
            self.start_rotate()
        else:
            self.get_logger().error('Move fehlgeschlagen')
            self.mission_success = False
            self.end_reason = "MOVE_FAILED"
            self.done_event.set()
        
    def send_feedback(self, state):
        '''
        Sendet aktuelle Phase als Feedback an den Client.
        state: Aktuelle Phase als String ("ROTATE" oder "MOVE")
        '''
        if self.current_goal_handle is None:
            return
        
        feedback = HandlerAc.Feedback()
        feedback.current_phase = state
        #feedback.completed_cycles = self.current_cycle +1
        self.current_goal_handle.publish_feedback(feedback)
    

def main(args=None):
    rclpy.init(args=args)
    node = HandlerActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Handler Action Server wird heruntergefahren (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    