#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.get_logger().info(f"controller_node Started")

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(1/2, self.timer_cb)
        
    def timer_cb(self):
        msg = TwistStamped()
        
        msg.twist.linear.x = 1.0
        msg.twist.angular.z = 0.3
        
        self.pub.publish(msg)
        
        
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()