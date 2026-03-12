import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # 추가됨

def generate_launch_description():
    urdf_tutorial_share = FindPackageShare('urdf_tutorial')

    model_arg = DeclareLaunchArgument(
        name='model',
        description='Absolute path to robot urdf file'
    )

    default_rviz_config_path = PathJoinSubstitution([
        urdf_tutorial_share, 'rviz', 'urdf.rviz'
    ])
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    # 수정된 부분: ParameterValue로 Command 결과값을 감싸줍니다.
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])