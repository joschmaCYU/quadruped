import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'quadruped_basics'
    
    # Path to URDF
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'quad.urdf.xacro'
    )

    # Process Xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 3. Python IK Node
    ik_node = Node(
        package=pkg_name,
        executable='ik_node.py',
        name='quadruped_ik_node',
        output='screen'
    )

    # sudo pkill -9 micro_ros_agent
    kill_process = ExecuteProcess(
        cmd=[
            'pkill', '-9', 'micro_ros_agent'
        ],
        output='screen'
    )


    # 4. Micro-ROS Agent via Docker (Sans le flag -it)
    microros_agent_process = ExecuteProcess(
        cmd=[
            'docker', 'run', '--rm', 
            '-v', '/dev:/dev', '--privileged', '--net=host', 
            'microros/micro-ros-agent:jazzy', 
            'serial', '--dev', '/dev/ttyUSB0'
        ],
        output='screen'
    )

    # ros2 launch ldlidar_stl_ros2 ld19.launch.py
    ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB1'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        # {'angle_crop_min': 135.0},
        # {'angle_crop_max': 225.0}
        ]
      )

    base_link_to_laser_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_ld06',
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0.18',
                '--roll', '0', 
                '--pitch', '0', 
                '--yaw', '0',
                '--frame-id', 'base_footprint', 
                '--child-frame-id', 'base_laser'
            ]
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

# ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_nav2.yaml
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'map': '/home/ros/ros2_ws/src/quadruped_basics/maps/third_better_map.yaml',
            'params_file': '/home/ros/ros2_ws/src/quadruped_basics/config/my_nav2.yaml'
        }.items()
    )

    #ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_slam_params.yaml use_sim_time:=false
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': '/home/ros/ros2_ws/src/quadruped_basics/config/my_slam_params.yaml'
        }.items()
    )



    return LaunchDescription([
        rsp_node,
        ik_node,
        nav2_rviz_node,
        ldlidar_node,
        base_link_to_laser_tf_node,
        kill_process,
        microros_agent_process
    ])
