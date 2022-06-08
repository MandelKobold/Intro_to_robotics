import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.left_distance = 0
        self.right_distance = 0
        self.goal = None
        self.position = None
        self.position_old = None
        self.position_new = None
        self.heading = None
        self.vector_to_goal = None
        self.angle = None
        self.count = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.get_logger().info('Herbert START')
        
    def timer_cb(self):
        # Winkelgeschwindigkeit: winkel / Zeit (1s?)
        # Idee: Heading berechnen indem wir geradeaus fahren und dann berechnen wohin wir gefahren sind, dahin gucken wir
        # dann wenn wir das goal haben, den winkel zwischen gucken und goal berechnen und uns drehen mit passender winkelgeschwindigkeit
        # danach oder währenddessen noch mal geradeaus fahren und das neue heading berechnen 
        # damit neuen winkel berechnen und wieder drehen
        # repeat bis goal erreicht ist

        # Fahr nach vorne und fahr nicht gegen Wände
        msg = Twist()
        rirarotation_left = 2.7
        rirarotation_right = -2.7
        left = self.left_distance - 0.3
        right = self.right_distance -0.3
        wall = min(left,right)

        x = self.forward_distance - 0.3
        x = x if x < 0.1 else 0.1
        x = x if x >= 0 and wall >= 0 else 0.0

        #msg.linear.x = x
        #if self.count % 10 == 0:
        msg.linear.x = 0.05

        if x == 0.0:
            if left < right:
                msg.angular.z = rirarotation_right
            else:
                msg.angular.z = rirarotation_left

        #Wohin guckst du? 
        if self.position_old != None and self.position_new != None:
            self.heading = np.subtract(self.position_new,self.position_old)     #Vektor von alter zu neuer Position
        if self.position_new != None and self.goal != None:
            self.vector_to_goal = np.subtract(self.goal,self.position_new)                #Vektor von neuer position zu Ziel
        self.get_logger().info(f'Herbert GOAL: @ {self.goal}s')
        self.get_logger().info(f'Herbert HEADING: @ {self.heading}s')
        self.get_logger().info(f'Herbert GOAL Vektor: @ {self.vector_to_goal}s')

        #Winkel zum Ziel berechnen
        if isinstance(self.heading, np.ndarray) and isinstance(self.vector_to_goal, np.ndarray):
            self.angle = np.arccos(np.dot(self.heading,self.vector_to_goal)/(np.absolute(self.heading) @ np.absolute(self.vector_to_goal)))  #Winkel zwischen aktueller Position und Ziel Vektoren
            if self.angle > 0.05:
                msg.angular.z = -0.3
            if self.angle > 3.14:
                msg.angular.z = 0.3
        self.get_logger().info(f'Herbert ANGLE: @ {self.angle}s')

        #Zum Goal drehen (wenn wir Richtung goal gucken sind wir fertig mit drehen)
        
        self.count = self.count + 1
        
        self.publisher.publish(msg)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]	#Wie weit ist die Wand vor mir weg
        #von den Sensoren 30° vorne jeweils den mit der geringsten Abstand zur Wand
        self.left_distance = min(msg.ranges[:30])
        self.left_distance = self.left_distance if self.left_distance > msg.range_min and self.left_distance < msg.range_max else 5
        self.right_distance = min(msg.ranges[-30:])
        self.right_distance = self.right_distance if self.right_distance > msg.range_min and self.left_distance < msg.range_max else 5
        #self.get_logger().info(f'Herbert Position2: @ {self.position}s')
        
    def position_cb(self, msg):
        self.position_old = self.position
        self.position = msg.point.x, msg.point.y
        self.position_new = self.position
        self.get_logger().info(f'Herbert Position: @ {self.position_old}s')
        self.get_logger().info(f'DEBUG @ {self.position_new }')
        self.get_logger().info(f'Herbert Position: @ {self.position}s')

       


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
