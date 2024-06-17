#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.declare_parameter('wheel_base', 0.297)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('max_pwm', 255)

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.left_motor_pub = self.create_publisher(Int32, 'motor/left_cmd', 10)
        self.right_motor_pub = self.create_publisher(Int32, 'motor/right_cmd', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

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

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [self.left_wheel_vel, self.right_wheel_vel]

        # Here you would update the wheel positions and velocities based on actual encoder readings
        # For example:
        # self.left_wheel_pos = read_left_encoder()
        # self.right_wheel_pos = read_right_encoder()
        # self.left_wheel_vel = calculate_velocity(self.left_wheel_pos)
        # self.right_wheel_vel = calculate_velocity(self.right_wheel_pos)

        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
