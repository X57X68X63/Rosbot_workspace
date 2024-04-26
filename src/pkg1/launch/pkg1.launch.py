import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from geometry_msgs.msg import Twist
import time
 
 
 
def generate_launch_description():
    
    find_object_2d_launch = os.path.join(
        get_package_share_directory('aiil_rosbot_demo'), 'launch', 'find_object_2d.launch.py'
    )
 
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                find_object_2d_launch
            ]),
            launch_arguments={'some_argument': 'value'}.items(),
        ),
 
        Node(
            package='pkg1',
            executable='object_detection_mover',
            name='object_detection_mover',
            output='screen'
        ),
 
        Node(
            package='pkg1',
            executable='exploration_manager',
            name='exploration_manager',
            output='screen'
        )
        
    ])