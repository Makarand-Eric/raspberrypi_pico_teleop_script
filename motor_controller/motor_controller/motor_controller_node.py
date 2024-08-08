import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('Motor Controller Node has been started.')

    def listener_callback(self, msg):
        # Convert the Twist message to motor commands
        linear_speed = int(msg.linear.x * 100)  # Adjust scale factor as needed
        angular_speed = int(msg.angular.z * 100)  # Adjust scale factor as needed

        left_motor_speed = linear_speed - angular_speed
        right_motor_speed = linear_speed + angular_speed

        # Ensure the speeds are within the acceptable range
        left_motor_speed = max(min(left_motor_speed, 255), -255)
        right_motor_speed = max(min(right_motor_speed, 255), -255)

        # Send commands to the Pico
        command_left = f'1,{left_motor_speed}\n'
        command_right = f'2,{right_motor_speed}\n'
        
        self.ser.write(command_left.encode('utf-8'))
        self.ser.write(command_right.encode('utf-8'))
        
        self.get_logger().info(f'Sent command: {command_left.strip()} {command_right.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

