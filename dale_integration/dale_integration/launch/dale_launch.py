from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Launch external launch files ---

        # Motion Planning (MoveIt config)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py'
                ])
            ]),
            launch_arguments={'ur_type': 'ur3e', 'launch_rviz': 'false'}.items()
        ),

        # Motion Execution (Control stack)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur3_control'),
                    'launch',
                    'moveit_stack.launch.py'
                ])
            ]),
            launch_arguments={'ur_type': 'ur3e', 'launch_rviz': 'false'}.items()
        ),

        # Image Processing Launch File - TO BE ADDED
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('img_prc'),
        #             'launch',
        #             'img_prc.launch.py'
        #         ])
        #     ])
        # ),

        # Tool Path Planner Launch File - TO BE ADDED
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('tool_path_planning'),
        #             'launch',
        #             'tool_path.launch.py'
        #         ])
        #     ])
        # ),

        # --- Direct node example (uncomment and configure as needed) ---

        # Node(
        #     package='your_package',
        #     executable='your_node_executable',
        #     name='your_node_name',
        #     output='screen',
        #     parameters=[{'example_param': 'value'}]
        # ),
    ])