import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paket dizinlerini al
    package_dir = get_package_share_directory('create3_rpi_pkg')
    laser_filters_dir = get_package_share_directory('laser_filters')
    
    # navigation için launch dosyasını dahil et
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rplidar_a1.launch.py')
        )
    )

    #laser_filters için launch dosyasını dahil et
    laser_filters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_filters_dir, 'examples', 'range_filter_example.launch.py')
        )
    )

    # statik dönüşüm yayıcı düğümü tanımla
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0.13", "0", "0.05", "3.141592", "0", "0", "base_link", "laser"]
    )

    # LaunchDescription döndür
    return LaunchDescription([
        navigation,
        static_transform_publisher_node,
        laser_filters
    ])
