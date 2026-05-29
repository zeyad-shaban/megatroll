#!/usr/bin/env python3
from megajaw_brain import constants
import rclpy
from rclpy.node import Node
from megajaw_brain.utils import clip_num
from megajaw_interfaces.msg import TargetControl
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import enum


class STATES(enum.Enum):
    IDLE = 0
    TO_TARGET = 1
    GRIPPER_CLOSE = 2
    GO_HOME = 3
    GRIPPER_OPEN = 4


class ToTargetControllerNode(Node):
    def __init__(self):
        super().__init__("to_target_controller_node")
        self.get_logger().info(f"to_target_controller_node Started")

        self.declare_parameter("is_autonomous", True)
        self.is_autonomous = self.get_parameter("is_autonomous").value

        self.W_MAX = 0.7
        self.KW = self.W_MAX

        self.GAMMA = 1.3
        self.V_MAX = 0.5

        self.state = STATES["IDLE"]

        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.create_subscription(TargetControl, "/target_state", self.on_target_state, 10)
        self.create_timer(0.1, self.main_loop)

        self.target_pos: TargetControl | None = None

    def main_loop(self):
        if self.state == STATES.IDLE:
            self.idle()
        elif self.state == STATES.TO_TARGET:
            self.to_target()
        elif self.state == STATES.GRIPPER_CLOSE:
            self.gripper_close()
        elif self.state == STATES.GO_HOME:
            pass
        elif self.state == STATES.GRIPPER_OPEN:
            pass

    # Main States
    def idle(self):
        # Open Gripper
        msg = Float64MultiArray()
        msg.data = [0.6, -0.6]
        self.gripper_pub.publish(msg)

        # Is there visible confirmed tar(msg.err_x, msg.err_y)get?
        if self.target_pos is not None:
            self.get_logger().info("Changing State IDLE -> TO_TARGET...")
            self.state = STATES.TO_TARGET

    def to_target(self):
        if self.target_pos is None:
            self.state = STATES.IDLE
            self.get_logger().warn("No target position available, going back to IDLE")
            return

        w = clip_num(self.KW * self.target_pos.err_x, -self.W_MAX, self.W_MAX)
        v = clip_num(self.V_MAX * (1 - abs(self.target_pos.err_x)) ** (self.target_pos.depth / 2), -self.V_MAX, self.V_MAX)

        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = v
        cmd_msg.twist.angular.z = w

        self.cmd_vel_pub.publish(cmd_msg)

        if self.target_pos.depth < constants.GRIPPER_CLOSE_DEPTH:
            self.get_logger().info("Changing State TO_TARGET -> GRIPPER_CLOSE...")
            self.state = STATES.GRIPPER_CLOSE

    def gripper_close(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.gripper_pub.publish(msg)

        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Utility
    def on_target_state(self, msg: TargetControl):
        self.target_pos = msg


def main(args=None):
    rclpy.init(args=args)
    node = ToTargetControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
