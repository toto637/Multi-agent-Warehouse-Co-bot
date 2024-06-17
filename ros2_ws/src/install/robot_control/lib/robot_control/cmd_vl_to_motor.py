#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('max_pwm', 255)

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.left_motor_pub = self.create_publisher(Int32, 'motor/left_cmd', 10)
        self.right_motor_pub = self.create_publisher(Int32, 'motor/right_cmd', 10)

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        left_velocity = linear_velocity - (angular_velocity * self.wheel_base / 2)
        right_velocity = linear_velocity + (angular_velocity * self.wheel_base / 2)

        left_pwm = self.velocity_to_pwm(left_velocity)
        right_pwm = self.velocity_to_pwm(right_velocity)

        self.publish_motor_commands(left_pwm, right_pwm)

    def velocity_to_pwm(self, velocity):
        max_wheel_speed = self.max_pwm * self.wheel_radius  # Maximum speed of the wheel in m/s
        pwm = int((velocity / max_wheel_speed) * self.max_pwm)
        return max(min(pwm, self.max_pwm), -self.max_pwm)

    def publish_motor_commands(self, left_pwm, right_pwm):
        left_msg = Int32()
        right_msg = Int32()

        left_msg.data = left_pwm
        right_msg.data = right_pwm

        self.left_motor_pub.publish(left_msg)
        self.right_motor_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
