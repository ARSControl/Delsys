from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'timer_value',
            default_value='60',
            description='Timer value in seconds'
        ),
        DeclareLaunchArgument(
            'timer_flag',
            default_value='False',
            description='Flag to enable/disable timer'
        ),
        
        # Node configuration
        Node(
            package='py_socket',
            executable='reader',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'timer_value': LaunchConfiguration('timer_value'),
                 'timer_flag': LaunchConfiguration('timer_flag')}
            ]
        )
    ])

