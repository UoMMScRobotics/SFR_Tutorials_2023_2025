import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'example_urdf_robot'
    file_subpath = 'urdf/diff_drive.urdf.xacro'


    # # Set ignition resource path (so it can find your world files)
    # ign_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH',
    # value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    
    # https://github.com/clearpathrobotics/clearpath_simulator/blob/fe89d458e5cc64c775194737270185c104beff3e/clearpath_gz/launch/gz_sim.launch.py#L46
    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[':'.join(packages_paths)])


    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # # Add features
    # gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    # arguments=['-file', sdf_path,
    # '-x', '2.0',
    # '-y', '0.5',
    # '-z', '0.0'],
    # output='screen'
    # )
 
    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')


    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/model/diff_drive_example/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
                    '/model/diff_drive_example/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
                    '/model/diff_drive_example/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
                    '/model/diff_drive_example/tf'       + '@tf2_msgs/msg/TFMessage'    + '[' + 'ignition.msgs.Pose_V',
                    '/model/diff_drive_example/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',
                    '/model/diff_drive_example/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    ],
        parameters= [{'qos_overrides./diff_drive_example.subscriber.reliability': 'reliable'}],
        remappings= [
                    ('/model/diff_drive_example/cmd_vel',  '/cmd_vel'),
                    ('/model/diff_drive_example/odometry', '/odom'   ),
                    ('/model/diff_drive_example/scan',     '/scan'   ),
                    ('/model/diff_drive_example/tf',       '/tf'     ),
                    ('/model/diff_drive_example/imu',      '/imu_raw'),
                    ('/model/diff_drive_example/joint_state', 'joint_states')
                    ],
        output='screen'
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=[]
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(launch_gazebo)
    # ld.add_action(gz_spawn_objects)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(node_rviz)
    return ld
