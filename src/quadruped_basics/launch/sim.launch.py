import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- UPDATE THIS TO MATCH YOUR PACKAGE NAME ---
    pkg_name = 'quadruped_basics'
    
    # 1. Locate the URDF/XACRO file
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'quad.urdf.xacro')
    
    # 2. Process XACRO into a raw XML string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Start the Robot State Publisher (Publishes the physical layout to the network)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'box_fort.sdf')

    # 4. Start Gazebo Simulator (Loading an empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 5. Spawn the robot inside Gazebo
    # NOTE: We spawn it at z=0.5 meters high so it drops safely to the floor
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'quadruped', 
            '-allow_renaming', 'true', 
            '-z', '0.5'
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    # 6. Turn on the Broadcaster (Reads the joints)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # 7. Turn on the Position Controller (Listens to Python and moves the virtual motors)
    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_group_position_controller'],
        output='screen'
    )

    ik_node = Node(
        package=pkg_name,
        executable='ik_node.py',
        name='quadruped_ik_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nav2_rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    nav2_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

# ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_nav2.yaml
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/home/ros/ros2_ws/src/quadruped_basics/maps/third_better_map.yaml',
            'params_file': '/home/ros/ros2_ws/src/quadruped_basics/config/my_nav2.yaml'
        }.items()
    )

    #ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_slam_params.yaml use_sim_time:=true
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': '/home/ros/ros2_ws/src/quadruped_basics/config/my_slam_params.yaml'
        }.items()
    )

    delayed_actions = TimerAction(
        period=5.0,
        actions=[nav2_cmd, slam_toolbox_cmd]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        clock_bridge,
        # Wait for the robot to spawn BEFORE turning on the broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        # Wait for the broadcaster to start BEFORE turning on the motors
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),
        ik_node,
        nav2_rviz_node,
        delayed_actions,

    ])
