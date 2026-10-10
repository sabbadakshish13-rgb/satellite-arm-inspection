from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_dir = Path(get_package_share_directory("satellite_arm_description"))
    gazebo_dir = Path(get_package_share_directory("ros_gz_sim"))

    world_file = package_dir / "worlds" / "inspection_zero_gravity.sdf"
    robot_file = package_dir / "urdf" / "satellite_arm.urdf"
    target_file = package_dir / "models" / "inspection_target" / "model.sdf"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_dir / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items()
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "inspection_world",
            "-name", "satellite_arm",
            "-file", str(robot_file)
        ],
        output="screen"
    )

    spawn_target = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "inspection_world",
            "-name", "inspection_target",
            "-file", str(target_file),
            "-x", "0.65",
            "-y", "0.10",
            "-z", "0.75"
        ],
        output="screen"
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot, spawn_target]
    )


    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/satellite_arm/camera/image"],
        output="screen"
    )

    joint1_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/satellite_arm/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double"
        ],
        output="screen"
    )

    joint2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/satellite_arm/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double"
        ],
        output="screen"
    )


    tracking_controller = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", str(Path.home() / "auto_marker_tracking.py")],
                output="screen"
            )
        ]
    )

    return LaunchDescription([gazebo, delayed_spawn, camera_bridge, joint1_bridge, joint2_bridge, tracking_controller])
