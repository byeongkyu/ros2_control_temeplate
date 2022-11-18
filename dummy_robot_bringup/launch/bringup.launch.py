from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('dummy_robot_description'),
            'urdf/robot.urdf.xacro',
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('dummy_robot_bringup'),
            "config",
            "controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('dummy_robot_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
        }.items()
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )

    load_emo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'emo_controller'],
        output='screen'
    )

    ld.add_action(upload_robot)
    ld.add_action(control_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_base_controller)
    ld.add_action(load_emo_controller)
    return ld