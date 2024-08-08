import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial

class LCUTalker(Node):
    def __init__(self):
        super().__init__('lcu_talker')
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        self.timer2 = self.create_timer(0.2, self.read_command)

        self.nav_status_sub = self.create_subscription(
            Int32,
            'diagnostics/navigation_status',
            self.navigation_state_callback,
            10)
        
        self.soft_es_status_sub = self.create_subscription(
            Int32,
            'software_es/status',
            self.software_es_state_callback,
            10)
        
        self.soft_es_lidar_sub = self.create_subscription(
            Twist,
            'software_es/lidar_safety',
            self.lidar_safety_state_callback,
            10)
       
        self.ser = serial.Serial( '/dev/ttyACM1',
                     baudrate = 115200,
                     stopbits = serial.STOPBITS_ONE,
                     bytesize = serial.EIGHTBITS,
                     writeTimeout = 0,
                     timeout = 0,
                     rtscts = True,
                     dsrdtr = True )

        self.get_logger().info('Serial port opened with baud rate 115200')

        self.last_linear_x = 0.55
        self.command = {
            'move_cmd': [0.0, 0.0],
            'emergency_status': -1,
            'lidar_safety': [0, 0, 0, 0, 0, 0],
            'navigation_status': -1
        }

    def twist_callback(self, msg):
        self.linear_x = msg.linear.x
        # Detect rapid increase in angular velocity as a proxy for 'Q' key press
        if msg.linear.x < self.last_linear_x :
            self.update_command('move_cmd', [msg.linear.x, msg.angular.z])  # Threshold can be adjusted
           
        else:
        # Always update command with current velocities
            self.ser.flush()
            self.get_logger().info('Serial buffer flushed on pressing Q.')            
        
    def navigation_state_callback(self, msg):
        self.update_command('navigation_status', msg.data)
    
    def software_es_state_callback(self, msg):
        self.update_command('emergency_status', msg.data)
    
    def lidar_safety_state_callback(self, msg):
        self.update_command('lidar_safety', [msg.front, msg.back, 
                                             msg.front_left, msg.front_right, 
                                             msg.back_left, msg.back_right])

    def update_command(self, key, value):
        self.command[key] = value
        self.send_command()
    
    def read_command(self):
        data = self.ser.readline()
        self.ser.flush()
        self.get_logger().info('Recieved command: {}'.format(data))

    def send_command(self):
        command = f"{self.command['move_cmd'][0]},{self.command['move_cmd'][1]}," \
                  f"{self.command['emergency_status']}," \
                  f"{self.command['navigation_status']}," \
                  f"{self.command['lidar_safety'][0]},{self.command['lidar_safety'][1]}," \
                  f"{self.command['lidar_safety'][2]},{self.command['lidar_safety'][3]}," \
                  f"{self.command['lidar_safety'][4]},{self.command['lidar_safety'][5]}"
        self.ser.write((command + '\n').encode('utf-8'))
        #self.read_command()
        self.ser.flush()
        #self.get_logger().info('Sent command: {}'.format(command))

def main(args=None):
    rclpy.init(args=args)
    node = LCUTalker()
    rclpy.spin(node)
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
