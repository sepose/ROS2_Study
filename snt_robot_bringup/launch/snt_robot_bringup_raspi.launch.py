import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_pkg = get_package_share_directory("snt_robot_config")

    # 1. 하드웨어 본체 (방금 만든 파일)
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(config_pkg, "launch", "moveit_rviz.launch.py"))
    )

    return LaunchDescription([
        moveit_rviz
    ])