import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_pkg = get_package_share_directory("snt_robot_config")

    # 1️⃣ HW 인터페이스 런치
    hw_launch_file = os.path.join(config_pkg, "launch", "snt_ros2_control.launch.py")

    # HW 런치 먼저 실행
    try:
        import launch
        from launch_ros.actions import Node

        hw_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_launch_file)
        )
        print("[INFO] Starting HW interface launch...")
    except Exception as e:
        print(f"[ERROR] Failed to include HW launch: {e}")
        sys.exit(1)

    # 2️⃣ 후속 런치들 (HW 초기화 성공 시)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg, "launch", "rsp.launch.py")
        )
    )
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg, "launch", "move_group.launch.py")
        )
    )
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg, "launch", "spawn_controllers.launch.py")
        )
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg, "launch", "moveit_rviz.launch.py")
        )
    )

    # 3️⃣ 전체 LaunchDescription
    # Note: HW 런치 실패 시 IncludeLaunchDescription 자체가 실패하면 Launch 종료
    return LaunchDescription([
        LogInfo(msg="[INFO] Launching HW interface..."),
        hw_launch,
        LogInfo(msg="[INFO] HW interface included. Launching MoveIt and related nodes..."),
        rsp_launch,
        static_tf_launch,
        move_group_launch,
        spawn_controllers_launch,
        moveit_rviz_launch
    ])