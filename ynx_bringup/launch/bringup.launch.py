from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    ip = context.launch_configurations['ip']
    port = context.launch_configurations['port']
    ns = context.launch_configurations['ns']
    use_ft_sensor = context.launch_configurations['use_ft_sensor']
    netft_ip = context.launch_configurations['netft_ip']
    cpf = context.launch_configurations['cpf']
    cpt = context.launch_configurations['cpt']
    use_mock_hardware = context.launch_configurations["use_mock_hardware"]
    log_level = context.launch_configurations["log_level"]
    launch_rviz = context.launch_configurations["launch_rviz"]
    launch_servo = context.launch_configurations["launch_servo"]

    print("")
    print("Starting bringup with paramaters:")
    print(" log_level:           " + log_level)
    print(" ip:                  " + ip)
    print(" port:                " + port)
    if use_ft_sensor == "true":
        print(" netft_ip:            " + netft_ip)
        print(" cpf:                 " + cpf)
        print(" cpt:                 " + cpt)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" use_mock_hardware:   " + use_mock_hardware)
    print(" launch_servo:        " + launch_servo)
    print(" launch_rviz:         " + launch_rviz)
    print("")

    driver_launch_path = PathJoinSubstitution([FindPackageShare('ynx_bringup'), 'launch', 'driver.launch.py'])
    moveit_launch_path = PathJoinSubstitution([FindPackageShare('ynx_bringup'), 'launch', 'moveit.launch.py'])
    # robot_manager_launch_path = PathJoinSubstitution([FindPackageShare('ynx_bringup'), 'launch', 'robot_manager.launch.py'])

    driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch_path),
            launch_arguments={
                'log_level': log_level,
                'ip': ip,
                'port': port,
                'ns': ns,
                'use_mock_hardware': use_mock_hardware,
                'use_ft_sensor': use_ft_sensor,
                'netft_ip': netft_ip,
                'cpf': cpf,
                'cpt': cpt,
                }.items()
            )

    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
            launch_arguments={
                'log_level': log_level,
                'ns': ns,
                'launch_rviz': launch_rviz,
                'launch_servo': launch_servo,
                }.items()
            )

    # robot_manager = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(robot_manager_launch_path),
    #         launch_arguments={
    #             'log_level': log_level,
    #             'ns': ns,
    #             }.items()
    #         )

    # return [driver, moveit, robot_manager]
    return [driver, moveit]

def generate_launch_description():
    # add launch arguments
    declared_arguments = []
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
    declared_arguments.append(
            DeclareLaunchArgument(
                "use_ft_sensor",
                default_value="true",
                description="Use ATI ft sensor",
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "netft_ip", 
                default_value="192.168.19.210",
                description="IP address by which the sensor can be reached."
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "cpf", 
                default_value="600000",
                description="counts per force"
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "cpt", 
                default_value="1000000",
                description="counts per torque"
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
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            )
    declared_arguments.append(
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Moveit Servo?"),
            )
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

