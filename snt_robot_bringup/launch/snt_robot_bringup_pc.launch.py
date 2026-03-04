import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_pkg = get_package_share_directory("snt_robot_config")

    # 1. 하드웨어 본체 (방금 만든 파일)
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "snt_ros2_control.launch.py"))
    )

    # 2. MSA가 만든 4가지 자체실행 런치들
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "rsp.launch.py"))
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "static_virtual_joint_tfs.launch.py"))
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "move_group.launch.py"))
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "spawn_controllers.launch.py"))
    )

    return LaunchDescription([
        hardware_interface,
        rsp,
        static_tf,
        move_group,
        spawn_controllers,
    ])
