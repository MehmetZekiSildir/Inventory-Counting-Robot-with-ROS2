from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rfid_pub',
            executable='inventory_bag_recorder',
            output='screen',
            parameters=[{"deployment_name": 'ebebek'},
                        {"bag_file_name": "test"},],
            emulate_tty=True),
            
    ])