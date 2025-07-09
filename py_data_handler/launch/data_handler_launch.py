from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments for flags
    info_flag_arg = DeclareLaunchArgument(
        'info_flag', default_value='False',
        description='Flag to enable saving of INFO data.'
    )
    emg_flag_arg = DeclareLaunchArgument(
        'emg_flag', default_value='False',
        description='Flag to enable saving of EMG data.'
    )
    acc_flag_arg = DeclareLaunchArgument(
        'acc_flag', default_value='False',
        description='Flag to enable saving of ACC data.'
    )
    gyro_flag_arg = DeclareLaunchArgument(
        'gyro_flag', default_value='False',
        description='Flag to enable saving of GYRO data.'
    )

    # Declare launch arguments for directories
    # Default is current working directory, which the node also does.
    current_working_directory = os.getcwd()
    info_dir_arg = DeclareLaunchArgument(
        'info_dir', default_value=current_working_directory,
        description='Directory to save INFO data.'
    )
    emg_dir_arg = DeclareLaunchArgument(
        'emg_dir', default_value=current_working_directory,
        description='Directory to save EMG data.'
    )
    acc_dir_arg = DeclareLaunchArgument(
        'acc_dir', default_value=current_working_directory,
        description='Directory to save ACC data.'
    )
    gyro_dir_arg = DeclareLaunchArgument(
        'gyro_dir', default_value=current_working_directory,
        description='Directory to save GYRO data.'
    )

    # Use LaunchConfiguration to get the values
    info_flag = LaunchConfiguration('info_flag')
    emg_flag = LaunchConfiguration('emg_flag')
    acc_flag = LaunchConfiguration('acc_flag')
    gyro_flag = LaunchConfiguration('gyro_flag')
    info_dir = LaunchConfiguration('info_dir')
    emg_dir = LaunchConfiguration('emg_dir')
    acc_dir = LaunchConfiguration('acc_dir')
    gyro_dir = LaunchConfiguration('gyro_dir')

    # Node configuration
    data_handler_node = Node(
        package='py_data_handler',
        executable='handler',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'info_flag': info_flag},
            {'emg_flag': emg_flag},
            {'acc_flag': acc_flag},
            {'gyro_flag': gyro_flag},
            {'info_dir': info_dir},
            {'emg_dir': emg_dir},
            {'acc_dir': acc_dir},
            {'gyro_dir': gyro_dir},
        ]
    )

    return LaunchDescription([
        info_flag_arg,
        emg_flag_arg,
        acc_flag_arg,
        gyro_flag_arg,
        info_dir_arg,
        emg_dir_arg,
        acc_dir_arg,
        gyro_dir_arg,
        data_handler_node
    ])
