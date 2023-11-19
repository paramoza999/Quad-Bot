import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'p5'
    file_subpath = 'description/hyperdog.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Declare a launch argument for the Gazebo world file path
    world_file_path = DeclareLaunchArgument(
        'world',
        default_value='/home/param/new_ws/src/p5/worlds/test.world',
        description='Path to Gazebo world file'
    )

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]  # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    gazebo_joint_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_joint_controller", "-c", "/controller_manager"],
    )



    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'hyperdog'],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        world_file_path,  # Add the declared launch argument
        gazebo,
        node_robot_state_publisher,
        joint_controller,
        gazebo_joint_control,
        spawn_entity,
 
    ])
