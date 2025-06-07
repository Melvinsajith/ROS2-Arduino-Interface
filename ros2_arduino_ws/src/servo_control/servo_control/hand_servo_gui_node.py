import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import mediapipe as mp
import tkinter as tk
import threading
import serial
import time

# Serial setup - change this to your Arduino port
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    time.sleep(2)
except Exception as e:
    print("Warning: Serial port not opened:", e)
    ser = None

# MediaPipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

angle = 90
lock = threading.Lock()

class HandServoNode(Node):
    def __init__(self):
        super().__init__('hand_servo_node')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)

        self.hand_control_enabled = True

        self.root = tk.Tk()
        self.root.title("Servo Control GUI")

        self.label = tk.Label(self.root, text="Angle: 90°")
        self.label.pack(pady=10)

        self.slider = tk.Scale(self.root, from_=0, to=180, orient=tk.HORIZONTAL, length=300)
        self.slider.set(90)
        self.slider.pack(pady=10)

        # Buttons for fixed angles
        angle_frame = tk.Frame(self.root)
        angle_frame.pack(pady=5)
        for a in [0, 45, 90, 135, 180]:
            tk.Button(angle_frame, text=f"{a}°", width=6,
                      command=lambda ang=a: self.set_angle(ang)).pack(side=tk.LEFT, padx=3)

        # Nudge buttons
        nudge_frame = tk.Frame(self.root)
        nudge_frame.pack(pady=5)
        tk.Button(nudge_frame, text="−5°", width=6, command=self.decrease_angle).pack(side=tk.LEFT, padx=5)
        tk.Button(nudge_frame, text="+5°", width=6, command=self.increase_angle).pack(side=tk.LEFT, padx=5)

        # Stop and Resume buttons
        tk.Button(self.root, text="🛑 STOP Hand Control", bg="red", fg="white", command=self.stop_hand_control).pack(pady=8)
        tk.Button(self.root, text="▶️ Resume Hand Control", bg="green", fg="white", command=self.resume_hand_control).pack(pady=5)

        # Close program button
        tk.Button(self.root, text="❌ Close Program", bg="gray", fg="white", command=self.close_program).pack(pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.close_program)  # Handle window close button

        self.update_gui()
        self.send_loop()

        threading.Thread(target=self.camera_loop, daemon=True).start()

    def set_angle(self, ang):
        global angle
        with lock:
            angle = ang
        self.slider.set(ang)
        self.label.config(text=f"Angle: {ang}°")

    def increase_angle(self):
        global angle
        with lock:
            angle = min(180, angle + 5)
        self.slider.set(angle)

    def decrease_angle(self):
        global angle
        with lock:
            angle = max(0, angle - 5)
        self.slider.set(angle)

    def stop_hand_control(self):
        self.get_logger().info("Hand tracking disabled.")
        self.hand_control_enabled = False

    def resume_hand_control(self):
        self.get_logger().info("Hand tracking resumed.")
        self.hand_control_enabled = True

    def update_gui(self):
        with lock:
            current_angle = angle
        self.slider.set(current_angle)
        self.label.config(text=f"Angle: {current_angle}°")
        self.root.after(100, self.update_gui)

    def send_loop(self):
        with lock:
            current_angle = angle
        # Send to serial
        if ser and ser.is_open:
            try:
                ser.write(f"{current_angle}\n".encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")

        # Publish to ROS 2 topic
        msg = Int32()
        msg.data = current_angle
        self.publisher.publish(msg)

        self.root.after(200, self.send_loop)

    def camera_loop(self):
        global angle
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                continue

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            if results.multi_hand_landmarks and self.hand_control_enabled:
                for hand_landmarks in results.multi_hand_landmarks:
                    x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x
                    new_angle = int(x * 180)
                    with lock:
                        angle = new_angle
                    mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Tracking", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit camera window
                break

        cap.release()
        cv2.destroyAllWindows()

    def close_program(self):
        self.get_logger().info("Closing program...")
        if ser and ser.is_open:
            ser.close()
        rclpy.shutdown()
        self.root.destroy()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandServoNode()
    node.root.mainloop()
    node.destroy_node()

if __name__ == '__main__':
    main()
