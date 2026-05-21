from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition
import os
import yaml
import xacro

def launch_setup(context, *args, **kwargs):
    model = context.launch_configurations['model']
    ns = context.launch_configurations['ns']
    log_level = context.launch_configurations['log_level']
    launch_servo = context.launch_configurations['launch_servo']
    launch_rviz = context.launch_configurations['launch_rviz']
    tf_prefix = context.launch_configurations["tf_prefix"]

    print("")
    print("Starting moveit with parameters:")
    print(" log_level:           " + log_level)
    print(" model:               " + model)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" launch_servo:        " + launch_servo)
    print(" launch_rviz:         " + launch_rviz)
    print("")

    config_package = 'ynx_bringup'

    # SRDF
    srdf_file_path = os.path.join(get_package_share_directory(config_package), 'srdf', model, model+'.srdf.xacro')
    srdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                srdf_file_path,
                " ",
                "tf_prefix:=",
                tf_prefix,
                ]
            )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(srdf_content, value_type=str)}

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

    # Planning Pipeline
    pilz_config_path = os.path.join(get_package_share_directory(config_package), 'config', 'pilz_industrial_motion_planner_planning.yaml')
    with open(pilz_config_path, 'r') as file:
        pilz_config_yaml = yaml.safe_load(file)
    planning_pipeline = {
        "planning_pipelines": {
            'pipeline_names': ["pilz_industrial_motion_planner"]
            },
        'pilz_industrial_motion_planner': pilz_config_yaml,
    }

    # Cartesian Limits
    cartesian_limits_path = os.path.join(get_package_share_directory(config_package), 'config', 'pilz_cartesian_limits.yaml')
    with open(cartesian_limits_path, 'r') as file:
        cartesian_limits_yaml = yaml.safe_load(file)
    cartesian_limits = {'robot_description_planning': cartesian_limits_yaml}

    # Trajectory Execution
    moveit_controllers_path = os.path.join(get_package_share_directory(config_package), 'config', 'moveit_controllers.yaml')
    trajectory_execution = ParameterFile(moveit_controllers_path, allow_substs=True)

    # MoveGroup Parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': False,
        'publish_geometry_updates': False,
        'publish_state_updates': False,
        'publish_transforms_updates': False,
        'publish_robot_description': False,
        'publish_robot_description_semantic': False,
    }
    move_group_capabilities = {
        "disable_capabilities": "".join([
            "move_group/ClearOctomapService",
        ]) 
    }
    disable_occupancy_map = {
        "sensors": [""],          # Do not load any 3D sensor plugins
        "octomap_frame": "",    # Clear the reference frame for the octomap
        "octomap_resolution": 0.0,
    }

    # MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=ns,
        output='screen',
        parameters=[
            robot_description_semantic,
            kinematics,
            joint_limits,
            planning_pipeline,
            cartesian_limits,
            trajectory_execution,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            disable_occupancy_map,
            {'use_sim_time': False} 
        ],
        arguments=[
            '--ros-args', 
            '--log-level', 
            'test.moveit.moveit.ros.occupancy_map_monitor:=FATAL',
            '--log-level', 
            log_level
        ]
    )

    # Servo
    servo_parameters_path = os.path.join(get_package_share_directory(config_package), 'config', 'servo_parameters.yaml')
    servo_parameters = ParameterFile(servo_parameters_path, allow_substs=True)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        namespace=ns,
        condition=IfCondition(launch_servo),
        parameters=[
            robot_description_semantic,
            kinematics,
            joint_limits,
            planning_pipeline,
            cartesian_limits,
            trajectory_execution,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            disable_occupancy_map,
            servo_parameters,
        ],
        output="screen",
        arguments=[
            '--ros-args', 
            '--log-level', 
            log_level
        ]
    )

    # RVIZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ynx_bringup"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        namespace=ns,
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args', '--log-level', log_level],
        parameters=[
            robot_description_semantic,
            kinematics,
            {
                "use_sim_time": False,
            },
        ],
    )

    return [move_group_node, servo_node, rviz_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='nex10',
            description="Type/series of used YNX robot used.",
            choices=[
                "nex10",
            ],
            )
        )
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
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            )
    declared_arguments.append(
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Moveit Servo?"),
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
