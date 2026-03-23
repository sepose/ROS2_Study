from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("SNTRobot", package_name="snt_robot_config").to_moveit_configs()

    # ros2_control_node 본체 실행 (spawner들이 붙을 서버)

    # 런치파일 수정 (에러 안 나는 안전한 방식)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
         # 1. 로봇 모델 정보 (가장 먼저)
         moveit_config.robot_description,
         # 2. 기타 설정들...
         
         # 3. 승찬 님의 YAML 파일을 '맨 마지막'에 배치 (덮어쓰기 방지)
         os.path.join(moveit_config.package_path, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )
    return LaunchDescription([ros2_control_node])