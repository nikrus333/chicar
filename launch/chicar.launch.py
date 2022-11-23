from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
import os
import launch_ros

def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    frame_id = LaunchConfiguration('frame_id', default='scan')
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='chicar').find('chicar')
    default_model_path = os.path.join(pkg_share, 'description/urdf/chicar.urdf')

    return LaunchDescription([
        
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': port, 'frame_id': frame_id}],
            output='screen'),
        
        Node(
            package='chicar',
            executable='cmd_serial',
           ),
        Node(
            package='chicar',
            executable='wheel_odom',
            ),
        Node(
            package='chicar',
            executable='teleop',
            output='screen')
    ])
