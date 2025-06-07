from setuptools import setup

package_name = 'servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Servo control over serial using ROS 2 and Arduino',
    license='MIT',
    entry_points={
    'console_scripts': [
        'servo_node = servo_control.servo_node:main',
        'servo_gui = servo_control.gui_node:main',
        'servo_slider_gui = servo_control.gui_slider_node:main',
        'hand_servo_gui = servo_control.hand_servo_gui_node:main',
        'jetson_connect_hand = servo_control.jetson_connect_hand:main',

    ],
},
)

