import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk

class ServoControlGUI(Node):
    def __init__(self):
        super().__init__('servo_control_gui')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)
        self.current_angle = 90

        # Create tkinter window
        self.root = tk.Tk()
        self.root.title("Servo Control")

        self.label = tk.Label(self.root, text="Angle: 90°")
        self.label.pack(pady=10)

        self.slider = tk.Scale(self.root, from_=0, to=180, orient=tk.HORIZONTAL,
                               length=300, command=self.slider_changed)
        self.slider.set(90)
        self.slider.pack(pady=10)

        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)
        for angle in [0, 45, 90, 135, 180]:
            tk.Button(btn_frame, text=f"{angle}°", width=6,
                      command=lambda a=angle: self.set_angle(a)).pack(side=tk.LEFT, padx=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start the repeating publish loop
        self.schedule_publish()

    def slider_changed(self, value):
        self.current_angle = int(value)
        self.label.config(text=f"Angle: {self.current_angle}°")

    def set_angle(self, angle):
        self.slider.set(angle)
        self.current_angle = angle
        self.label.config(text=f"Angle: {angle}°")

    def publish_angle(self):
        msg = Int32()
        msg.data = self.current_angle
        self.publisher.publish(msg)
        self.get_logger().info(f"Published angle: {self.current_angle}")

    def schedule_publish(self):
        # Run ROS timers/events and publish continuously
        rclpy.spin_once(self, timeout_sec=0)
        self.publish_angle()
        self.root.after(200, self.schedule_publish)  # call again in 200 ms

    def on_close(self):
        self.get_logger().info("Shutting down GUI...")
        rclpy.shutdown()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlGUI()
    node.root.mainloop()
    node.destroy_node()

if __name__ == '__main__':
    main()
