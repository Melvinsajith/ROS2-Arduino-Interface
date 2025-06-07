# ROS2-Arduino-Interface


🔧 ROS2-Arduino-Interface

This project enables communication between a ROS 2-based system (such as Jetson Nano or a PC) and an Arduino microcontroller to control a servo motor. It includes multiple interfaces for user input—buttons, sliders, and hand-tracking via webcam—to publish servo angles over ROS 2 and send them to the Arduino via serial communication.
🗂 Project Structure
File	Description
servo_node.py	ROS 2 node that subscribes to servo angle topics and sends angle commands to Arduino via serial.
gui_node.py	Simple Tkinter GUI with angle buttons that publish servo angles.
gui_slider_node.py	Enhanced GUI with a slider and buttons for finer angle control.
hand_servo_gui_node.py	GUI and webcam-based hand tracking using MediaPipe to control servo. Includes manual override.
jetson_connect_hand.py	Similar to hand_servo_gui_node.py, tailored for Jetson devices with better GUI controls and error handling.
📡 Communication Flow

    User Input → via GUI Buttons / Slider / Hand Gestures

    ROS 2 Node publishes angle (std_msgs/Int32) on topic /servo_angle.

    servo_node.py subscribes to /servo_angle and transmits it to Arduino through serial (e.g., /dev/ttyUSB0).

    Arduino interprets and moves the servo accordingly.

🖥 Interface Modes
1. gui_node.py

    Buttons for discrete angles (0°, 45°, 90°, 135°, 180°).

    Lightweight and easy to use.

2. gui_slider_node.py

    Adds a slider for continuous control.

    Also includes angle buttons.

    Publishes servo angle at regular intervals (every 200 ms).

3. hand_servo_gui_node.py

    Combines MediaPipe hand tracking and GUI.

    Hand position (x-axis of index finger) maps to servo angle (0°–180°).

    Supports pausing/resuming hand control and manual fine-tuning.

4. jetson_connect_hand.py

    GUI and camera-activated control with a polished interface using ttk.

    Designed for Jetson Nano or similar platforms.

    Tracks wrist x-coordinate to control servo.

    Integrates real-time slider updates.

🤖 servo_node.py (Arduino Connector)

    Listens to /servo_angle topic.

    Sends validated angles to Arduino as serial strings (e.g., 90\n).

    Ensures angles are within 0–180° range.

🧠 Dependencies

    ROS 2 (rclpy)

    tkinter for GUI

    mediapipe for hand gesture tracking

    opencv-python for camera feed

    pyserial for Arduino communication

🚀 Getting Started

    Flash Arduino with a basic servo control sketch.

    Connect Arduino via USB (/dev/ttyUSB0).

    Launch ROS 2 and run desired interface node:

    ros2 run <your_package> servo_node
    ros2 run <your_package> gui_node  # or gui_slider_node, etc.

🔌 Arduino Example Sketch

#include <Servo.h>
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(9);
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
    }
  }
}
