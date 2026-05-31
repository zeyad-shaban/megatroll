#!/usr/bin/env python3
from megajaw_brain import constants
import rclpy
from rclpy.node import Node
from megajaw_brain.utils import clip_num
from megajaw_interfaces.msg import TargetControl
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import enum
import time


class STATES(enum.Enum):
    IDLE = 0
    TO_TARGET = 1
    GRIPPER_CLOSE = 2
    GO_HOME = 3
    GRIPPER_OPEN = 4


# Todo handle losing target
class ToTargetControllerNode(Node):
    def __init__(self):
        super().__init__("to_target_controller_node")
        self.get_logger().info("to_target_controller_node Started")

        self.declare_parameter("close_thresh", 0.04)
        self.close_thresh = self.get_parameter("close_thresh").value

        self.declare_parameter("W_MAX", 0.7)
        self.declare_parameter("KW", 0.7)
        self.declare_parameter("V_MAX", 0.6)
        self.declare_parameter("KV", 1.3)

        self.W_MAX = self.get_parameter("W_MAX").value
        self.KW = self.get_parameter("KW").value
        self.V_MAX = self.get_parameter("V_MAX").value
        self.KV = self.get_parameter("KV").value

        self.state = STATES["IDLE"]

        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.create_subscription(
            TargetControl, "/target_state", self.on_target_state, 10
        )
        self.create_timer(0.1, self.main_loop)

        self.target_ctrl: TargetControl | None = None

        self.state_enter_time: None | float = None
        self.forward_duration = 0.0

    def main_loop(self):
        if self.state == STATES.IDLE:
            self.idle()
        elif self.state == STATES.TO_TARGET:
            self.to_target()
        elif self.state == STATES.GRIPPER_CLOSE:
            self.gripper_close()
        elif self.state == STATES.GO_HOME:
            self.go_home()
        elif self.state == STATES.GRIPPER_OPEN:
            self.gripper_open()

    # Main States
    def idle(self):
        # Open Gripper
        msg = Float64MultiArray()
        msg.data = [1.0, -1.0]
        self.gripper_pub.publish(msg)

        # Is there visible confirmed tar(msg.err_x, msg.err_y)get?
        if self.target_ctrl is not None and self.target_ctrl.target_detected:
            self.get_logger().info("Changing State IDLE -> TO_TARGET...")
            self.state = STATES.TO_TARGET
            self.state_enter_time = time.monotonic()

    def to_target(self):
        if self.target_ctrl is None or not self.target_ctrl.target_detected:
            self.state = STATES.IDLE
            self.get_logger().warn("No target position available, going back to IDLE")
            return

        w = clip_num(self.KW * self.target_ctrl.err_x, -self.W_MAX, self.W_MAX)
        v = clip_num(self.KV * self.target_ctrl.depth, 0.0, self.V_MAX)

        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = v
        cmd_msg.twist.angular.z = w

        self.cmd_vel_pub.publish(cmd_msg)

        if self.target_ctrl.depth < self.close_thresh:
            self.get_logger().info("Changing State TO_TARGET -> GRIPPER_CLOSE...")
            self.state = STATES.GRIPPER_CLOSE

            assert self.state_enter_time is not None, (
                "Error self.state_enter_time cannot be None..."
            )
            self.forward_duration = time.monotonic() - self.state_enter_time
            self.state_enter_time = time.monotonic()

    def gripper_close(self):
        # Stop Car
        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        # Close Gripper
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.gripper_pub.publish(msg)

        # todo add delay if required
        self.get_logger().info("Changing State GRIPPER_CLOSE -> GO_HOME...")
        self.state = STATES.GO_HOME

    def go_home(self):
        # Go back same amount of seconds
        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = -0.5
        cmd_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        assert self.state_enter_time is not None, (
            "Error self.state_enter_time cannot be None..."
        )

        elapsed_back = time.monotonic() - self.state_enter_time
        remaining = self.forward_duration - elapsed_back

        if remaining <= 0:
            self.get_logger().info("Changing State GO_HOME -> GRIPPER_OPEN...")
            self.state_enter_time = None
            self.state = STATES.GRIPPER_OPEN

    def gripper_open(self):
        # Stop Car
        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        # Open Gripper
        msg = Float64MultiArray()
        msg.data = [1.0, -1.0]
        self.gripper_pub.publish(msg)

        self.get_logger().info("Changing State GRIPPER_OPEN -> IDLE...")
        self.state = STATES.IDLE
        self.target_ctrl = None

    # Utility
    def on_target_state(self, msg: TargetControl):
        self.target_ctrl = msg


def main(args=None):
    rclpy.init(args=args)
    node = ToTargetControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
