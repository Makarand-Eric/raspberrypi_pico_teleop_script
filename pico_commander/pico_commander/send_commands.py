import rclpy
from rclpy.node import Node
import serial
import time

class PicoCommander(Node):
    def __init__(self):
        super().__init__('pico_commander')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600
        self.ser = None
        self.open_serial_port()

        # Send commands at regular intervals
        self.timer = self.create_timer(2.0, self.send_command)

    def open_serial_port(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info('Serial port {} opened with baud rate {}'.format(self.serial_port, self.baud_rate))
        except serial.SerialException as e:
            self.get_logger().error('Could not open serial port {}: {}'.format(self.serial_port, e))
            self.create_timer(5.0, self.open_serial_port)  # Retry after 5 seconds

    def send_command(self):
        if self.ser and self.ser.is_open:
            command = '1,50\n'  # Example command
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info('Sent command: {}'.format(command.strip()))
        else:
            self.get_logger().warn('Serial port not open')

def main(args=None):
    rclpy.init(args=args)
    node = PicoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

