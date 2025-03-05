#!/usr/bin/env python3
"""
Launch the TurtleBot3 (Waffle) with a Mars-themed world and D* planner.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

print("Running buildmap.py from:", __file__)

def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # WORLD: Use custom Mars-themed world located in turtlebot/models/mars_world.world
    worldfile = os.path.join(pkgdir('turtlebot'),
                             'models', 'mars_world.world')
    # MODEL: Locate the Gazebo TurtleBot model: Using the standard model.
    modelfile = os.path.join(pkgdir('turtlebot'), 'models', 'turtlebot.sdf')
    cleanlidarmodelfile = os.path.join(pkgdir('turtlebot'), 'models', 'turtlebot_cleanlidar.sdf')
    noisylidarmodelfile = os.path.join(pkgdir('turtlebot'), 'models', 'turtlebot_noisylidar.sdf')

    # URDF: Locate and load the TurtleBot URDF.
    urdffile = os.path.join(pkgdir('turtlebot'), 'urdf', 'turtlebot.urdf')
    with open(urdffile, 'r') as file:
        robot_description = file.read()

    # MAP: Locate the good map
    mapfile = os.path.join(pkgdir('turtlebot'), 'maps/goodmap.yaml')

    # RVIZ: Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('turtlebot'), 'rviz/viewturtlebot.rviz')

    # BAGS: Locate the BAG folder.
    bagfolder = os.path.join(pkgdir('turtlebot'), 'bags', 'leftside_cleanlaser')

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    ### SIMULATION.
    # Gazebo Server with the Mars-themed world file.
    incl_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkgdir('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': worldfile}.items())

    # Gazebo Client (optional; uncomment if you want to see the Gazebo GUI)
    incl_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkgdir('gazebo_ros'), 'launch', 'gzclient.launch.py')))

    # Spawn the TurtleBot (using the noisy lidar version here)
    node_spawn_turtlebot_noisylidar = Node(
        name       = 'spawn_turtlenbot',
        package    = 'gazebo_ros',
        executable = 'spawn_entity.py',
        output     = 'screen',
        arguments  = ['-entity', 'waffle', '-file', noisylidarmodelfile,
                      '-x', '-2.0', '-y', '1.0', '-z', '0.01'])

    ### SIMULATION ALTERNATIVE
    # ROS Bag Playback (comment out if using Gazebo)
    cmd_playback = ExecuteProcess(
            cmd    = ['ros2', 'bag', 'play', '--clock', '10', bagfolder],
            output = 'screen')

    ### RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        parameters = [{'use_sim_time': True}],
        on_exit    = Shutdown())

    ### URDF PROCESSING
    # Robot State Publisher to publish the TF frames using the URDF.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description},
                      {'use_sim_time': True}])

    ### LOCALIZATION
    # Perfect localization using a static transform publisher (map to odom).
    node_perfectlocalization = Node(
        name       = 'localization',
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments  = ['--frame-id', 'map', '--child-frame-id', 'odom'])

    ### MAPPING
    # Build the map using the custom buildmap node.
    node_buildmap = Node(
        name       = 'buildmap',
        package    = 'turtlebot',
        executable = 'buildmap',
        output     = 'screen',
        parameters = [{'use_sim_time': True}])

    ### D* PLANNER
    # Launch the D* planner node.
    node_dstar_planner = Node(
        name       = 'd_star_planner',
        package    = 'turtlebot',
        executable = 'd_star_planner',
        output     = 'screen')

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST

    return LaunchDescription([
        # Start Gazebo with the Mars-themed world.
        incl_gzserver,
        # Optionally, start Gazebo client by uncommenting the line below:
        # incl_gzclient,
        # Spawn the TurtleBot.
        node_spawn_turtlebot_noisylidar,
        # Uncomment the following if you want to run bag playback instead of Gazebo:
        # cmd_playback,
        # Start RViz for visualization.
        node_rviz,
        # Publish robot state using URDF.
        node_robot_state_publisher,
        # Publish the static transform for localization.
        node_perfectlocalization,
        # Run the buildmap node to construct the map.
        node_buildmap,
        # Launch the D* planner node.
        node_dstar_planner,
    ])

if __name__ == '__main__':
    generate_launch_description()
