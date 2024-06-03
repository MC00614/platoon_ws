import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_nodes(context, *, num_trucks):
    nodes = []

    # ros_param_file = os.path.join(
    #         get_package_share_directory('longitudianl_control'),
    #         'config',
    #         'config.yaml') 

    for i in range(int(num_trucks) + 1):
        node = Node(
            package='longitudianl_control',
            executable='velocity_control',
            output='screen',
            # parameters=[ros_param_file],
            arguments=[
                f'--truck_id={i}',
            ],
            on_exit=launch.actions.Shutdown()  
        )
        nodes.append(node)
        
        if i == 0:
            continue
        
        node = Node(
            package='longitudianl_control',
            executable='distance_control',
            output='screen',
            # parameters=[ros_param_file],
            arguments=[
                f'--truck_id={i}',
            ],
            on_exit=launch.actions.Shutdown()  
        )
        nodes.append(node)
            
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    return generate_nodes(context, num_trucks=num_trucks)


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
