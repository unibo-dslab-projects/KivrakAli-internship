from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# --- Add/remove robots here ---
ROBOTS = [
    {'name': 'robot1', 'x': 0.0, 'y': 0.0,  'yaw': 0.0},
    {'name': 'robot2', 'x': 0.0, 'y': 1.5,  'yaw': 0.0},
    {'name': 'robot3', 'x': 0.0, 'y': -1.5, 'yaw': 0.0},
]

def generate_launch_description():
    pkg = FindPackageShare('swarm_sim')
    xacro_file = PathJoinSubstitution([pkg, 'models', 'diff_robot.urdf.xacro'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    # '-s' runs the server only (headless). Override gz_args to get the GUI.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    # Single global clock bridge so ROS uses Gazebo sim time
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    nodes = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r -s -v4 empty.sdf'),
        gz_sim, clock_bridge,
    ]

    for i, r in enumerate(ROBOTS):
        ns = r['name']
        robot_desc = Command(['xacro ', xacro_file, ' namespace:=', ns])

        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': use_sim_time,
                         'frame_prefix': ns + '/'}],
        )
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', ns, '-topic', ['/', ns, '/robot_description'],
                       '-x', str(r['x']), '-y', str(r['y']),
                       '-z', '0.15', '-Y', str(r['yaw'])],
        )
        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            arguments=[
                ['/', ns, '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
                ['/', ns, '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
            ],
        )
        nodes += [rsp, bridge]
        # Stagger spawns ~2 s apart: gives robot_state_publisher time to latch
        nodes.append(TimerAction(period=float(2 * (i + 1)), actions=[spawn]))

    return LaunchDescription(nodes)
