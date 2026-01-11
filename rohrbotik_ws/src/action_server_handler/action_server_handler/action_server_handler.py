from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import rclpy
from interfaces.action import HandlerAc, RotateAc, MoveAc
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node

class HandlerActionServer(Node):
    def __init__(self):
        super().__init__('handler_server_node')
        self.callback_group = ReentrantCallbackGroup()
        self.done_event = threading.Event()
        self.current_goal_handle = None
        self.handler_active = False
        self.target_vel = 0.0
        self.state = "IDLE"
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

        if not self.move_client.wait_for_server(timeout_sec = 5.0):
            self.get_logger().warn('Move Server nicht gefunden. Starte trotzdem')
        else:
            self.get_logger().info('Move Server gefunden')

        if not self.rotate_client.wait_for_server(timeout_sec = 5.0):
            self.get_logger().warn('Rotate Server nicht gefunden. Starte trotzdem')
        else:
            self.get_logger().info('Rotate Server gefunden')

        
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Neues Handler-Goal: target_vel={goal_request.target_vel:.2f}m/s')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
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
        self.target_vel = goal_handle.request.target_vel
        self.target_vel = max(0.0, min(0.2, self.target_vel))

        self.current_cycle = 0
        self.mission_success = True
        self.end_reason = ""
        self.state = "ROTATING"

        self.get_logger().info(f'Handler Mission starten mit target_vel={self.target_vel:.2f}m/s')

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
    
    def start_rotate(self):         #startet rotate action server
        if not self.handler_active:
            return
        
        self.get_logger().info('Starte ROTATE')
        self.send_feedback("ROTATE")
        rotate_goal = RotateAc.Goal()
        rotate_goal.one_direction = False

        send_goal_future = self.rotate_client.send_goal_async(rotate_goal,
                                                                feedback_callback=self.rotate_feedback_callback)
        
        send_goal_future.add_done_callback(self.rotate_goal_response_callback)

    def rotate_goal_response_callback(self, future):
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

    def rotate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback            #nicht nötig

    def rotate_result_callback(self, future):
        result = future.result().result

        if not self.handler_active:
            return
        
        if result.success:
            self.get_logger().info('Rotate erfolgreich - Rohr gefunden')
            self.state = "MOVING"
            self.start_move()
        else:
            self.get_logger().error('Rotate fehlgeschlagen - Rohr nicht gefunden')
            self.mission_success = False
            self.end_reason = "ROTATE_FAILED__NO_PIPE_FOUND"
            self.done_event.set()

    def start_move(self):
        if not self.handler_active:
            return
        
        self.get_logger().info(f'[Zyklus {self.current_cycle +1}] Starte MOVE')
        self.send_feedback("MOVE")

        move_goal = MoveAc.Goal()
        move_goal.target_vel = self.target_vel

        send_goal_future = self.move_client.send_goal_async(move_goal,
                                                            feedback_callback=self.move_feedback_callback)
        
        send_goal_future.add_done_callback(self.move_goal_response_callback)

    def move_goal_response_callback(self, future):
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
    
    def move_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
    
    def move_result_callback(self, future):
        result = future.result().result

        if not self.handler_active:
            return
        
        if result.success:
            self.get_logger().info('Move erfolgreich abgeschlossen - Wendeplattform erreicht')
            self.current_cycle += 1
            self.state = "ROTATING"
            self.start_rotate()
        else:
            self.get_logger().error('Move fehlgeschlagen')
            self.mission_success = False
            self.end_reason = "MOVE_FAILED"
            self.done_event.set()
        
    def send_feedback(self, state):
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
    except KeyboardInterrupt:
        node.get_logger().info('Handler Action Server wird heruntergefahren (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    