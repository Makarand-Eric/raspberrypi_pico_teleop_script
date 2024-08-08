import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.port = '/dev/ttyACM0'
        self.baudrate = 9600
        self.counter = 0  # Initialize the counter

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.port} opened with baudrate {self.baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
                self.counter += 1  # Increment the counter for each received message
                self.get_logger().info(f"Received ({self.counter}): {line}")
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
            else:
                self.get_logger().debug("No data waiting in the serial buffer.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading from serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
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

