import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess  # <-- NOUVEL IMPORT ICI
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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

    # 2. RViz2 Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # 3. Python IK Node
    ik_node = Node(
        package=pkg_name,
        executable='ik_node.py',
        name='quadruped_ik_node',
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

    return LaunchDescription([
        rsp_node,
        rviz_node,
        ik_node,
        microros_agent_process  # <-- AJOUTÉ À LA LISTE ICI
    ])
