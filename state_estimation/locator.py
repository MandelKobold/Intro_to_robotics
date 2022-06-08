import aifc
import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)           #update rate
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        
        # YOUR CODE GOES HERE:

        i = 0
        x = 0.0, 0.0, 0.0

        while i < 20: 
            x_dach = x

            mi = np.array([a.range for a in self.anchor_ranges])
            ai = np.array([[a.anchor.x, a.anchor.y, a.anchor.z]for a in self.anchor_ranges])

            NablaRix_dach = - (x_dach - ai)/np.linalg.norm(x_dach - ai)
            Rix_dach = mi - np.linalg.norm(x_dach - ai)

            x = x_dach-np.linalg.pinv(NablaRix_dach) @ Rix_dach

            i = i + 1
            #self.get_logger().info('Herbert LOCATOR')


        """
        while np.square(np.linalg.norm(mi - mi_dach)) > 0.1:
            x_dach = 0.0, 0.0, 0.0
            mi_dach = np.linalg.norm(x_dach-self.anchor_ranges.anchor)    
            Rix_dach = self.anchor_ranges.range - np.linalg.norm(x_dach-self.anchor_ranges.anchor)
            NablaRix_dach = - (x_dach - self.ranges.anchor)/np.linalg.norm(x_dach - self.ranges.anchor)
            x = x_dach-np.linalg.pinv(NablaRix_dach)*Rix_dach
            self.get_logger().info(f'Herbert Position: @ {x}s')

        #x = np.mean([r.range for r in self.anchor_ranges]) - 0.5
        #y = np.mean([r.range for r in self.anchor_ranges]) - 0.5
        #z = np.mean([r.range for r in self.anchor_ranges]) - 0.5
        """
        return x


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
