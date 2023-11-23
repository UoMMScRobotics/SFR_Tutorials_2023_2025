from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here



    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    return ld