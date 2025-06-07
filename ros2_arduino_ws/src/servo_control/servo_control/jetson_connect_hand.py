import cv2
import mediapipe as mp
from mediapipe.python.solutions.drawing_utils import draw_landmarks

import tkinter as tk
from tkinter import ttk, messagebox
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

angle = 90
lock = threading.Lock()

class HandServoGuiNode(Node):
    def __init__(self):
        super().__init__('hand_servo_gui')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)

        # GUI Setup
        self.root = tk.Tk()
        self.root.title("Hand Servo Control GUI")

        self.slider = ttk.Scale(self.root, from_=0, to=180, orient='horizontal', command=self.update_slider)
        self.slider.set(angle)
        self.slider.pack(pady=10, padx=10)

        # Buttons for fixed angles
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=5)
        for val in [0, 45, 90, 135, 180]:
            ttk.Button(button_frame, text=f"Set {val}", command=lambda v=val: self.set_angle(v)).pack(side=tk.LEFT, padx=5)

        ttk.Button(self.root, text="Stop (90°)", command=self.stop_servo).pack(pady=5)
        ttk.Button(self.root, text="Close", command=self.close_program).pack(pady=5)

        # Camera Setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            messagebox.showerror("Error", "Cannot open camera. Check device and permissions.")
            self.root.destroy()
            return

        # Hand tracking thread
        self.running = True
        threading.Thread(target=self.process_hand, daemon=True).start()

        # ROS publishing loop
        self.send_loop()

        self.root.protocol("WM_DELETE_WINDOW", self.close_program)
        self.root.mainloop()

    def update_slider(self, val):
        global angle
        with lock:
            angle = int(float(val))

    def set_angle(self, value):
        self.slider.set(value)

    def stop_servo(self):
        self.set_angle(90)

    def close_program(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.root.destroy()
        self.get_logger().info("Shutting down node...")
        rclpy.shutdown()

    def process_hand(self):
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

        while self.running:
            ret, image = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to grab frame")
                continue

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                wrist_x = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.WRIST].x
                with lock:
                    global angle
                    angle = int(wrist_x * 180)
                    self.root.after(0, lambda v=angle: self.slider.set(v))

            cv2.imshow('Hand Tracking', image)
            if cv2.waitKey(5) & 0xFF == 27:
                self.running = False
                break

        hands.close()
        self.cap.release()
        cv2.destroyAllWindows()

    def send_loop(self):
        with lock:
            current_angle = angle
        msg = Int32()
        msg.data = current_angle
        self.publisher.publish(msg)
        if self.running:
            self.root.after(200, self.send_loop)


def main(args=None):
    rclpy.init(args=args)
    HandServoGuiNode()


if __name__ == '__main__':
    main()
