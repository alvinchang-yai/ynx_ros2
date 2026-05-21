from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro

def launch_setup(context, *args, **kwargs):
    ns = context.launch_configurations['ns']
    log_level = context.launch_configurations['log_level']
    tf_prefix = context.launch_configurations["tf_prefix"]

    print("")
    print("Starting robot manager with parameters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print("")

    config_package = 'ynx_bringup'

    # Robot Description
    description_file = PathJoinSubstitution([FindPackageShare("nex10_description"), "urdf", "nex10.xacro"])
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

    # SRDF
    srdf_file_path = os.path.join(get_package_share_directory(config_package), 'srdf', 'nex10.srdf.xacro')
    srdf_config = xacro.process_file(srdf_file_path, mappings={"tf_prefix": tf_prefix})
    srdf = {'robot_description_semantic': srdf_config.toxml()}

    # Kinematics
    kinematics_path = os.path.join(get_package_share_directory(config_package), 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)
    kinematics = {'robot_description_kinematics': {f"{tf_prefix}manipulator": kinematics_yaml}}

    # Joint Limits (Planning constraints)
    joint_limits_path = os.path.join(get_package_share_directory(config_package), 'config', 'joint_limits.yaml')
    with open(joint_limits_path, 'r') as file:
        joint_limits_yaml = yaml.safe_load(file)
    raw_limits = joint_limits_yaml.get('joint_limits', {})
    prefixed_limits = {
            f"{tf_prefix}{joint_name}": limits 
            for joint_name, limits in raw_limits.items()
            }
    joint_limits = {'robot_description_planning': {"joint_limits": prefixed_limits}}

    # Moveit Arguments
    moveit_arguments = {
        'publish_planning_scene': False,
        'publish_geometry_updates': False,
        'publish_state_updates': False,
        'publish_transforms_updates': False,
        'publish_robot_description': False,
        'publish_robot_description_semantic': False,
    }

    # Robot Manager
    nex10_robot_manager = Node(
        package='nex10_robot_manager',
        executable='nex10_robot_manager',
        namespace=ns,
        output='screen',
        parameters=[
            robot_description,
            srdf,
            kinematics,
            joint_limits,
            moveit_arguments,
            {
                'ns': ns,
                'tf_prefix': tf_prefix,
            },
        ],
        arguments=[
            '--ros-args', 
            '--log-level', 
            log_level
        ]
    )

    return [nex10_robot_manager]


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
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
