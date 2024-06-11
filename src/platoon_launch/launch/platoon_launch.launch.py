from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    for package_name in 
    launch_file_path = os.path.join(
        get_package_share_directory('lane_follower'),
        'launch',
        'lane_follower.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_launch_file_path)
        ),
        # 주 launch 파일에서 추가로 실행할 노드
        Node(
            package='my_package',
            executable='another_node',
            name='my_main_node',
            output='screen',
            parameters=[{'paramA': 'valueA', 'paramB': 'valueB'}]
        )
    ])