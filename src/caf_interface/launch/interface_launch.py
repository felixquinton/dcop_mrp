import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('caf_sim_robots'), 'config', 'params.yaml')
    robot_id_1 = 'robot_1'

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='caf_interface', executable='interface_node', output='screen',
            name='interface_node',
            emulate_tty=True,
            parameters=[
                config
            ])
    ])

    return ld
