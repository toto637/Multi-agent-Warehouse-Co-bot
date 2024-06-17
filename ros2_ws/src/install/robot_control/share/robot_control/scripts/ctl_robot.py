import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

class MotorCommandNormalizer(Node):
    def __init__(self):
        super().__init__('motor_command_normalizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor/cmd',
            self.cmd_callback,
            10)
        self.left_motor_publisher = self.create_publisher(Int32, '/motor/left_cmd', 10)
        self.right_motor_publisher = self.create_publisher(Int32, '/motor/right_cmd', 10)
        
    def cmd_callback(self, msg):
        if len(msg.data) >= 2:
            left_motor_cmd = msg.data[0]
            right_motor_cmd = msg.data[1]

            # Normalize from range -4.18879 to 4.18879 to range -255 to 255
            left_motor_normalized = int((left_motor_cmd / 4.18879) * 255)
            right_motor_normalized = int((right_motor_cmd / 4.18879) * 255)

            # Create and publish messages
            left_msg = Int32()
            right_msg = Int32()
            left_msg.data = left_motor_normalized
            right_msg.data = right_motor_normalized

            self.left_motor_publisher.publish(left_msg)
            self.right_motor_publisher.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_command_normalizer = MotorCommandNormalizer()
    rclpy.spin(motor_command_normalizer)

    motor_command_normalizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
    

   
       