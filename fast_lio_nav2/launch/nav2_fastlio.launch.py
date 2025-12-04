from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('fast_lio_nav2'),
        'config',
        'nav2_fst.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server',
                ]
            }],
            output='screen'
        )
    ])
