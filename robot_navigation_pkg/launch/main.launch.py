import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # İlk launch dosyasının yolu
    rviz_file_dir = os.path.join(
        get_package_share_directory('robot_navigation_pkg'), 'rviz', 'tb3_navigation2.rviz')

    # İkinci launch dosyasının yolu
    bringup_file_dir = os.path.join(
        get_package_share_directory('robot_navigation_pkg'), 'launch', 'bringup_launch.py')
    

    # Include bringup launch file with map argument
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_file_dir)
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file_dir]  # Use the RViz config file provided as an argument
    )

    return LaunchDescription([
        bringup_launch,
        rviz_node
    ])
