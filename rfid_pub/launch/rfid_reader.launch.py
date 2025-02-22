from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rfid_pub',
            executable='rfid_reader_string_pub',
            output='screen',
            emulate_tty=True),
            
    ])