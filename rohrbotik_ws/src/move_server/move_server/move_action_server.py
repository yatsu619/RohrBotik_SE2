from .move_logic import PID












liner_vel,angular_vel=PID.zur_mitte_regeln
""" aufruf des reglers """



 cmd_aktuell = Twist()
""" publischen der geschwindeigkeit """
        cmd_aktuell.linear.x = linear_vel
        cmd_aktuell.angular.z = angular_vel
        self.cmd_pub.publish(cmd_aktuell)
