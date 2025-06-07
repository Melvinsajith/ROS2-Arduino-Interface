import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class ServoController(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        angle = msg.data
        if 0 <= angle <= 180:
            command = f"{angle}\n"
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f'Sent angle: {angle}')
        else:
            self.get_logger().warn('Angle out of range (0–180)!')

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
