import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    config_sim = Path(get_package_share_directory('caf_sim_robots'), 'config', 'params.yaml')
    # config_interface = Path(get_package_share_directory('caf_interface'), 'config', 'params.yaml')
    # package_prefix = get_package_share_directory('caf_robot')
    #
    # included_launch = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([package_prefix, '/three_robots_launch.py']))

    ld = launch.LaunchDescription()
    lc = launch.LaunchContext()
    launch.actions.DeclareLaunchArgument(name='pr', default_value='False').visit(lc)
    print(lc.launch_configurations['pr'])
    # ld.add_action(launch.actions.DeclareLaunchArgument(name='pr', default_value='False'))

    # if ld.get_launch_arguments():
    #     print('yeah')
    # else:
    #     print('ohh')

    ld.add_action(launch_ros.actions.Node(
            package='caf_sim_robots', executable='sim_robots_node', output='screen',
            name='simulation_robots_node',
            # emulate_tty is necessary to use print
            emulate_tty=True,
            # prefix=launch.substitutions.LaunchConfiguration('pr'),
            parameters=[
                config_sim
            ],
        ))

    return ld
