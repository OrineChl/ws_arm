from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('custom_3dof_description')
    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    description = ParameterValue(Command([FindExecutable(name='xacro'), ' "', model, '"']), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('model', default_value=PathJoinSubstitution([share, 'urdf', 'custom_3dof.urdf.xacro'])),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([share, 'rviz', 'custom_3dof.rviz'])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui), output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(use_gui), output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')], output='screen'),
    ])
