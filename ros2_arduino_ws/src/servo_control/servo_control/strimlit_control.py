import streamlit as st
import cv2
import mediapipe as mp
import numpy as np
from threading import Thread, Lock
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

angle = 90
lock = Lock()

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('streamlit_hand_servo')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)

    def publish_angle(self, val):
        msg = Int32()
        msg.data = val
        self.publisher.publish(msg)

# Initialize ROS 2
rclpy.init()
ros_node = ROS2Publisher()

def hand_tracking_thread():
    global angle
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    while st.session_state["running"]:
        ret, frame = cap.read()
        if not ret:
            continue
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            wrist_x = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.WRIST].x
            with lock:
                angle = int(wrist_x * 180)
                st.session_state["angle"] = angle

        # Store the frame to display
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        st.session_state["frame"] = frame_bgr

    cap.release()
    hands.close()

def ros_publish_loop():
    while st.session_state["running"]:
        with lock:
            current_angle = angle
        ros_node.publish_angle(current_angle)
        time.sleep(0.2)

# Streamlit GUI
if "running" not in st.session_state:
    st.session_state["running"] = True
    st.session_state["angle"] = 90
    st.session_state["frame"] = np.zeros((480, 640, 3), dtype=np.uint8)

    # Start threads
    Thread(target=hand_tracking_thread, daemon=True).start()
    Thread(target=ros_publish_loop, daemon=True).start()

st.title("🤖 Hand Servo Control (Streamlit + ROS2 + Mediapipe)")

st.subheader("Manual Angle Control")
angle_value = st.slider("Servo Angle", 0, 180, st.session_state["angle"])
with lock:
    angle = angle_value

col1, col2, col3, col4, col5 = st.columns(5)
for val, col in zip([0, 45, 90, 135, 180], [col1, col2, col3, col4, col5]):
    if col.button(f"Set {val}°"):
        with lock:
            angle = val
        st.session_state["angle"] = val

if st.button("Stop (90°)"):
    with lock:
        angle = 90
    st.session_state["angle"] = 90

if st.button("Close Program"):
    st.session_state["running"] = False
    ros_node.destroy_node()
    rclpy.shutdown()
    st.success("Shut down ROS and stopped threads.")
    st.stop()

st.subheader("Webcam Feed with Hand Tracking")
frame = st.session_state["frame"]
st.image(frame, channels="BGR", use_column_width=True)
