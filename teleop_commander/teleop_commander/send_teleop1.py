import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class TeleopCommander(Node):
    def __init__(self):
        super().__init__('teleop_commander')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.get_logger().info('Serial port {} opened with baud rate {}'.format(self.serial_port, self.baud_rate))

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info('Received command: linear_x={}, angular_z={}'.format(linear_x, angular_z))

        if linear_x > 0:
            self.send_command("forward")
        elif linear_x < 0:
            self.send_command("backward")
        elif angular_z > 0:
            self.send_command("left")
        elif angular_z < 0:
            self.send_command("right")
            # Optional: Handle left/right rotation based on angular_z
            pass
        else:
            self.send_command("stop")

    def send_command(self, command):
        self.ser.write((command + '\n').encode('utf-8'))
        self.get_logger().info('Sent command: {}'.format(command))

def main(args=None):
    rclpy.init(args=args)
    node = TeleopCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

