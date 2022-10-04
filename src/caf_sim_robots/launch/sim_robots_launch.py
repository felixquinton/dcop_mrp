import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_sim = Path(get_package_share_directory('caf_sim_robots'), 'config', 'params.yaml')

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='caf_sim_robots', executable='sim_robots_node', output='screen',
            name='simulation_robots_node',
            # emulate_tty is necessary to use print
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                config_sim
            ],
        ),
    ])

    # ld.add_action(node)

    return ld
