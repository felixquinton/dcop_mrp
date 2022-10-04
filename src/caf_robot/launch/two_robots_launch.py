from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    robot_id_1 = 'robot_1'
    robot_id_2 = 'robot_2'
    #  TODO optimize creation of robots by calling for i in range (1, nb_r) one_robot_launch

    return LaunchDescription([
        Node(
            package='caf_robot',
            namespace=robot_id_1,
            executable='robot_node',
            name=robot_id_1 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_1 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_1}
            ]
        ),
        Node(
            package='caf_robot',
            namespace=robot_id_1,
            executable='comsim_node',
            name=robot_id_1 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_1 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_1}
            ]
        ),
        Node(
            package='caf_robot',
            namespace=robot_id_2,
            executable='robot_node',
            name=robot_id_2 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_2 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_2}
            ]
        ),
        Node(
            package='caf_robot',
            namespace=robot_id_2,
            executable='comsim_node',
            name=robot_id_2 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_2 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_2}
            ]
        ),
    ])
