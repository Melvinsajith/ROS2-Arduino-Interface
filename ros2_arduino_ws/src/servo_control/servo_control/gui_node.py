import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk

class ServoGUI(Node):

    def __init__(self):
        super().__init__('servo_gui_node')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Servo Angle Controller")

        # Create buttons for servo angles
        for angle in [0, 45, 90, 135, 180]:
            btn = tk.Button(self.root, text=f"{angle}°", width=10,
                            command=lambda a=angle: self.publish_angle(a))
            btn.pack(pady=5)

        # Close ROS cleanly when GUI is closed
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_angle(self, angle):
        msg = Int32()
        msg.data = angle
        self.publisher.publish(msg)
        self.get_logger().info(f"Published angle: {angle}")

    def on_close(self):
        self.get_logger().info("Shutting down GUI node...")
        rclpy.shutdown()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    gui_node = ServoGUI()
    gui_node.root.mainloop()
    gui_node.destroy_node()

if __name__ == '__main__':
    main()
