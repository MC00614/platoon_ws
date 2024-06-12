from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launchs(context, *, num_trucks):
    launch_descriptions = []
    package_names = ['lane_detection', 'lane_follower', 'truck_detection', 'longitudianl_control']
    for package_name in package_names:
        launch_file_path = os.path.join(
            get_package_share_directory(package_name),
            'launch',
            f'{package_name}.launch.py'
        )
        launch_descriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments={'NumTrucks': num_trucks}.items()
            )
        )
    return launch_descriptions

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    return generate_launchs(context, num_trucks=num_trucks)


def generate_launch_description():

    declare_num_trucks = DeclareLaunchArgument(
        'NumTrucks',
        default_value='1',
        description='Number of trucks'
    )

    return LaunchDescription([
        declare_num_trucks,
        OpaqueFunction(function=launch_setup)
    ])
