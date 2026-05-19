#!/usr/bin/env python3
import struct
import time

import rclpy
import serial
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class UARTBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_pwm', 100)
        self.declare_parameter('min_pwm', 30)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('dead_zone', 0.02)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.dead_zone = float(self.get_parameter('dead_zone').value)

        self.max_pwm = int(self.clamp(self.max_pwm, 0, 100))
        self.min_pwm = int(self.clamp(self.min_pwm, 0, self.max_pwm))

        self.last_cmd_time = time.monotonic()
        self.stopped = True

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'UART connected to {self.serial_port}')
        except serial.SerialException as exc:
            self.ser = None
            self.get_logger().error(f'Failed to connect to UART: {exc}')

        self.create_subscription(TwistStamped, '/cmd_vel', self.twist_stamped_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('UART bridge node started')

    def twist_stamped_callback(self, msg):
        self.process_twist(msg.twist)

    def process_twist(self, twist):
        linear = self.clamp(float(twist.linear.x), -1.0, 1.0)
        angular = self.clamp(float(twist.angular.z), -1.0, 1.0)

        if abs(linear) < self.dead_zone:
            linear = 0.0
        if abs(angular) < self.dead_zone:
            angular = 0.0

        left, right = self.mix_differential(linear, angular)
        self.send_to_stm32(left, right)

        self.last_cmd_time = time.monotonic()
        self.stopped = left == 0 and right == 0

        self.get_logger().debug(
            f'linear.x={linear:.2f}, angular.z={angular:.2f}, '
            f'left_pwm={left}, right_pwm={right}'
        )

    def mix_differential(self, linear, angular):
        left = linear - angular
        right = linear + angular

        max_magnitude = max(1.0, abs(left), abs(right))
        left_pwm = self.scale_to_motor_pwm(left / max_magnitude)
        right_pwm = self.scale_to_motor_pwm(right / max_magnitude)

        return left_pwm, right_pwm

    def scale_to_motor_pwm(self, value):
        magnitude = abs(value)
        if magnitude < self.dead_zone:
            return 0

        pwm_range = self.max_pwm - self.min_pwm
        pwm = self.min_pwm + round(magnitude * pwm_range)
        signed_pwm = self.clamp(pwm, self.min_pwm, self.max_pwm)
        return int(signed_pwm * (1 if value > 0 else -1))

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def send_to_stm32(self, left_pwm, right_pwm):
        if self.ser is None:
            return

        left_pwm = int(self.clamp(left_pwm, -self.max_pwm, self.max_pwm))
        right_pwm = int(self.clamp(right_pwm, -self.max_pwm, self.max_pwm))

        try:
            message = struct.pack('<BBhh', 0xAA, 0x55, left_pwm, right_pwm)
            self.ser.write(message)
        except serial.SerialException as exc:
            self.get_logger().error(f'UART write error: {exc}')

    def timer_callback(self):
        if self.stopped:
            return

        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.send_to_stm32(0, 0)
            self.stopped = True
            self.get_logger().warn('cmd_vel timeout; sent motor stop')

    def destroy_node(self):
        self.send_to_stm32(0, 0)
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
