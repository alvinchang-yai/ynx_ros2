import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.substitutions import Command, PythonExpression
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def launch_setup(context):
    tf_prefix = context.launch_configurations['tf_prefix']
    model = context.launch_configurations['model']

    package_name = 'ynx_description'
    package_dir = get_package_share_directory(package_name) 
    description_file = os.path.join(package_dir, 'urdf', model, model+'.xacro')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    ))
    nodes.append(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    ))
    rviz_config_file = os.path.join(package_dir, 'rviz', 'display.rviz')
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    ))
    return nodes

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
            DeclareLaunchArgument(
                'ns',
                default_value='',
                description='namespace of the robot (used as prefix, so needed if running multiple robots)'
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"]))
            )
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='nex10',
            description="Type/series of used YNX robot.",
            choices=[
                "nex10",
            ],
            )
        )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
