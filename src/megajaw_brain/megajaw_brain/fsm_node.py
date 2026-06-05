#!/usr/bin/env python3
from megajaw_brain import constants
import rclpy
from rclpy.node import Node
from megajaw_brain.utils import clip_num
from megajaw_interfaces.msg import TargetControl
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import TwistStamped
import enum
import time


class STATES(enum.Enum):
    IDLE = 0
    TO_TARGET = 1
    ATTACH_ON_TARGET = 2
    GRIPPER_CLOSE = 3
    GO_HOME = 4
    GRIPPER_OPEN = 5


# Todo handle losing target
class ToTargetControllerNode(Node):
    def __init__(self):
        super().__init__("fsm_node")
        self.get_logger().info("fsm_node Started")

        self.declare_parameter("close_countdown_secs", 1.5)
        self.declare_parameter("close_thresh", 0.04)
        self.declare_parameter("W_MAX", 0.7)
        self.declare_parameter("KW", 0.7)
        self.declare_parameter("V_MAX", 0.6)
        self.declare_parameter("KV", 1.3)

        self.close_countdown_secs: float = self.get_parameter("close_countdown_secs").value  # type: ignore
        self.close_thresh: float = self.get_parameter("close_thresh").value  # type: ignore
        self.W_MAX: float = self.get_parameter("W_MAX").value  # type: ignore
        self.KW: float = self.get_parameter("KW").value  # type: ignore
        self.V_MAX: float = self.get_parameter("V_MAX").value  # type: ignore
        self.KV: float = self.get_parameter("KV").value  # type: ignore

        self.state = STATES["IDLE"]

        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.create_subscription(TargetControl, "/target_state", self.on_target_state, 10)
        self.create_subscription(Bool, "/auto_enabled", self.on_auto_enabled, 10)
        self.create_timer(1 / 30, self.main_loop)

        self.target_ctrl: TargetControl | None = None
        self.auto_enabled = False

        self.state_enter_time: None | float = None
        self.forward_duration = 0.0

        self.close_countdown_start: None | float = None
        
        msg = Float64MultiArray()
        msg.data = [1.0]
        self.gripper_pub.publish(msg)

    def main_loop(self):
        if not self.auto_enabled:
            self.state = STATES.IDLE
            self.state_enter_time = None
            return

        if self.state == STATES.IDLE:
            self.idle()
        elif self.state == STATES.TO_TARGET:
            self.to_target()
        elif self.state == STATES.ATTACH_ON_TARGET:
            self.attach_on_target()
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
        msg.data = [1.0]
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
            self.close_countdown_start = time.monotonic()
            self.get_logger().info("Changing State TO_TARGET -> ATTACH_ON_TARGET...")
            self.state = STATES.ATTACH_ON_TARGET

    def attach_on_target(self):
        assert self.close_countdown_start is not None, "self.close_countdown_start cannot be None..."
        remaining_time = self.close_countdown_secs - (time.monotonic() - self.close_countdown_start)

        self.get_logger().info(f"Target within close threshold, closing gripper in {remaining_time}")

        if remaining_time <= 0:
            self.get_logger().info("Changing State ATTACH_ON_TARGET -> GRIPPER_CLOSE...")
            self.state = STATES.GRIPPER_CLOSE

            assert self.state_enter_time is not None, "Error self.state_enter_time cannot be None..."
            
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
        msg.data = [0.0]
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

        assert self.state_enter_time is not None, "Error self.state_enter_time cannot be None..."

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
        msg.data = [1.0]
        self.gripper_pub.publish(msg)

        self.get_logger().info("Changing State GRIPPER_OPEN -> IDLE...")
        self.state = STATES.IDLE
        self.target_ctrl = None

    # Utility
    def on_target_state(self, msg: TargetControl):
        self.target_ctrl = msg

    def on_auto_enabled(self, msg: Bool):
        self.auto_enabled = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = ToTargetControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
