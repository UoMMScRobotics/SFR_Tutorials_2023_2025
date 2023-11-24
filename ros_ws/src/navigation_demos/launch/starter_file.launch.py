from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('navigation_demos')
    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])

    # Include Gazebo Simulation
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('gz_example_robot_description'), '/launch', '/sim_robot.launch.py']),
    launch_arguments={}.items(),
    )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={}.items(),
    )

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)

    return ld