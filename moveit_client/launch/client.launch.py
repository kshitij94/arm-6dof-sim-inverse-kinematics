from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm", package_name="moveit_config").to_moveit_configs()


    moveit_client_node = Node(
        package='moveit_client',
        executable='moveit_client_node',
        name='moveit_client_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    return LaunchDescription( [moveit_client_node])