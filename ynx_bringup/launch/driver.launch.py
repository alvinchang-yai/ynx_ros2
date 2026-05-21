from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    model = context.launch_configurations['model']
    ip = context.launch_configurations['ip']
    port = context.launch_configurations['port']
    # use_ft_sensor = context.launch_configurations['use_ft_sensor']
    # netft_ip = context.launch_configurations['netft_ip']
    # cpf = context.launch_configurations['cpf']
    # cpt = context.launch_configurations['cpt']
    use_mock_hardware = context.launch_configurations['use_mock_hardware']
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']

    print("")
    print("Starting bringup with paramaters:")
    print(" log_level:           " + log_level)
    print(" model:               " + model)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
        print(" port:                " + port)
    # if use_ft_sensor == "true":
    #     print(" netft_ip:            " + netft_ip)
    #     print(" cpf:                 " + cpf)
    #     print(" cpt:                 " + cpt)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" use_mock_hardware:   " + use_mock_hardware)
    # print(" use_ft_sensor:       " + use_ft_sensor)
    print("")

    pkg_name = "ynx_bringup"
    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", "ros2_controllers.yaml"]
    )
    description_file = PathJoinSubstitution([FindPackageShare("ynx_description"), "urdf", model, model+".xacro"])
    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                description_file,
                " ",
                "ip:=",
                ip,
                " ",
                "port:=",
                port,
                " ",
                # "use_ft_sensor:=",
                # use_ft_sensor,
                # " ",
                # "netft_ip:=",
                # netft_ip,
                # " ",
                # "cpf:=",
                # cpf,
                # " ",
                # "cpt:=",
                # cpt,
                # " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                ])
    robot_description = {
            "robot_description": ParameterValue(robot_description_content, value_type=str)
            }

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[robot_description]
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            ParameterFile(ros2_controllers_file, allow_substs=True),
            robot_description
            ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "--ros-args", "--log-level", log_level,
            ]
        ))
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_trajectory_controller",
            "--ros-args", "--log-level", log_level,
            ]
        ))
    # if use_ft_sensor == "true":
    #     nodes.append(Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace=ns,
    #         arguments=[
    #             "force_torque_sensor_broadcaster",
    #             "--ros-args", "--log-level", log_level,
    #             ]
    #         ))

    return nodes


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
            DeclareLaunchArgument(
                "ip", 
                default_value="192.168.19.201",
                description="IP address by which the robot can be reached."
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "port", 
                default_value="50300",
                description="Port by which the robot can be reached."
                )
            )
    # declared_arguments.append(
    #         DeclareLaunchArgument(
    #             "use_ft_sensor",
    #             default_value="true",
    #             description="Use ATI ft sensor",
    #             )
    #         )
    # declared_arguments.append(
    #         DeclareLaunchArgument(
    #             "netft_ip", 
    #             default_value="192.168.19.210",
    #             description="IP address by which the sensor can be reached."
    #             )
    #         )
    # declared_arguments.append(
    #         DeclareLaunchArgument(
    #             "cpf", 
    #             default_value="600000",
    #             description="counts per force"
    #             )
    #         )
    # declared_arguments.append(
    #         DeclareLaunchArgument(
    #             "cpt", 
    #             default_value="1000000",
    #             description="counts per torque"
    #             )
    #         )
    declared_arguments.append(
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
                )
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
