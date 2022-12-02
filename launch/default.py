import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(        
        get_package_share_directory('arpis_client'),
        # 'arpis_client',
        'urdf',
        urdf_file_name)        
    
    return LaunchDescription([        
        Node(
            package='arpis_client',            
            executable='arpis_main',
            name='arpis_client',                        
        ),
        Node(
            package='robot_state_publisher',            
            executable='robot_state_publisher',
            name='robot_state_publisher',          
            # parameters=[{'robot_description': 'robot'}],
            arguments=[urdf],            
            # output='screen',                        
        ),
#         Node(
#             package='rviz',
#             executable='rviz',
#             name='rviz',            
# #             arguments=['-d '+ os.path.join(get_package_share_directory('arpis_client'),
# # ('arpis_client'), 'rviz.rviz')],
#         ),
    ])