import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_sim = Path(get_package_share_directory('patrol_sim_robots'),
                      'config', 'params_ssi_demo.yaml')
    config_interface = Path(get_package_share_directory('patrol_interface'),
                            'config', 'params_ssi_demo.yaml')
    package_prefix = get_package_share_directory('patrol_robot')

    included_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_prefix, '/ssi_demo_two_robots_launch.py']))

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='patrol_sim_robots',
            executable='patrol_sim_robots_node', output='screen',
            name='simulation_robots_node',
            # emulate_tty is necessary to use print
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                config_sim
            ],
        ),
        launch_ros.actions.Node(
            package='patrol_interface',
            executable='patrol_interface_node', output='screen',
            name='interface_node',
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                config_interface,
            ]
        ),
        included_launch
    ])

    return ld
