from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder # Add this import

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("urdf", package_name="moveit_config")
        .robot_description(    file_path="config/urdf.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits(file_path="config/joint_limits.yaml") # Add this line
        .to_moveit_configs()
        )

    return LaunchDescription([
        Node(
            package='moveit_client',
            executable='moveit_client_node',
            name='moveit_client_node',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])