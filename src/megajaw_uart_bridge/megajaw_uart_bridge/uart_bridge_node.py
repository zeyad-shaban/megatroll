#!/usr/bin/env python3  
import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist, TwistStamped  
import serial  
import struct  
import time  
  
class UARTBridgeNode(Node):  
    def __init__(self):  
        super().__init__('uart_bridge_node')  
          
        # UART configuration  
        self.serial_port = '/dev/ttyAMA0'  # Change if needed  
        self.baudrate = 115200  
          
        # Robot parameters  
        self.wheel_separation = 0.18  # meters  
        self.wheel_radius = 0.03      # meters (adjust based on your wheels)  
          
        # Velocity limits  
        self.max_linear_vel = 1.0     # m/s  
        self.max_angular_vel = 1.0    # rad/s  
          
        # Timeout for stopping (seconds)  
        self.cmd_timeout = 0.5  
        self.last_cmd_time = time.time()  
          
        try:  
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)  
            self.get_logger().info(f'UART connected to {self.serial_port}')  
        except serial.SerialException as e:  
            self.get_logger().error(f'Failed to connect to UART: {e}')  
            self.ser = None  
          
        # Subscribe to cmd_vel (support both Twist and TwistStamped)  
        self.twist_sub = self.create_subscription(  
            Twist,  
            '/cmd_vel',  
            self.twist_callback,  
            10  
        )  
          
        self.twist_stamped_sub = self.create_subscription(  
            TwistStamped,  
            '/cmd_vel',  
            self.twist_stamped_callback,  
            10  
        )  
          
        # Timer for timeout check (20Hz)  
        self.timer = self.create_timer(0.05, self.timer_callback)  
          
        self.get_logger().info('UART Bridge Node started')  
  
    def twist_callback(self, msg):  
        self.process_twist(msg)  
  
    def twist_stamped_callback(self, msg):  
        self.process_twist(msg.twist)  
  
    def process_twist(self, twist):  
        self.last_cmd_time = time.time()  
          
        # Extract velocities  
        linear_x = twist.linear.x  
        angular_z = twist.angular.z  
          
        # Clamp velocities  
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))  
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))  
          
        # Differential drive kinematics  
        # v_left = (linear_x - angular_z * wheel_separation / 2) / wheel_radius  
        # v_right = (linear_x + angular_z * wheel_separation / 2) / wheel_radius  
          
        left_wheel_vel = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius  
        right_wheel_vel = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius  
          
        # Convert to PWM percentage (-100 to 100)  
        left_pwm = int(left_wheel_vel * 50)  # Adjust scaling factor as needed  
        right_pwm = int(right_wheel_vel * 50)  
          
        # Clamp PWM  
        left_pwm = max(-100, min(100, left_pwm))  
        right_pwm = max(-100, min(100, right_pwm))  
          
        self.send_to_stm32(left_pwm, right_pwm)  
          
        self.get_logger().debug(f'Linear: {linear_x:.3f}, Angular: {angular_z:.3f}, '  
                                f'Left PWM: {left_pwm}, Right PWM: {right_pwm}')  
  
    def send_to_stm32(self, left_pwm, right_pwm):  
        if self.ser is None:  
            return  
          
        # Send as binary: header (2 bytes) + left_pwm (2 bytes) + right_pwm (2 bytes)  
        # Header: 0xAA, 0x55  
        # PWM values: signed 16-bit integers  
        try:  
            message = struct.pack('<BBhh', 0xAA, 0x55, left_pwm, right_pwm)  
            self.ser.write(message)  
        except serial.SerialException as e:  
            self.get_logger().error(f'UART write error: {e}')  
  
    def timer_callback(self):  
        # Check for timeout and send stop command  
        if time.time() - self.last_cmd_time > self.cmd_timeout:  
            self.send_to_stm32(0, 0)  
  
    def destroy_node(self):  
        if self.ser:  
            self.send_to_stm32(0, 0)  
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
