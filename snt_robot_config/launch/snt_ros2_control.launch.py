from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("SNTRobot", package_name="snt_robot_config").to_moveit_configs()

    # ros2_control_node 본체 실행 (spawner들이 붙을 서버)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(moveit_config.package_path, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )

    return LaunchDescription([ros2_control_node])
