import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_sim = Path(get_package_share_directory('patrol_sim_robots'),
                      'config', 'params_inst_5r_20w_02.yaml')
    config_interface = Path(get_package_share_directory('patrol_interface'),
                            'config', 'params_inst_5r_20w_02.yaml')
    package_prefix = get_package_share_directory('patrol_robot')

    included_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            package_prefix, '/five_robots_inst_5r_20w_02_launch.py']))
    ind_file = open("./mission_logs/ind.txt", "r")
    print(ind_file)
    inst_id = ind_file.read()

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='patrol_sim_robots',
            executable='patrol_sim_robots_node',
            output='screen',
            name='simulation_robots_node',
            # emulate_tty is necessary to use print
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                {"team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_02.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_02.json",
                 "inst_id": inst_id}
            ],
        ),
        launch_ros.actions.Node(
            package='patrol_interface',
            executable='patrol_interface_node',
            output='screen',
            name='interface_node',
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                 {"robot_id": "interface_patrol",
                  "team_members_file_name": "team_spec_inst_5r_20w.json",
                  "robots_spec_file_name": "robots_spec_inst_5r_20w_02.json",
                  "mission_spec_file_path": "mission_spec_inst_5r_20w_02.json",
                  "inst_id": inst_id}
            ]
        ),
        launch_ros.actions.Node(
            package='patrol_essential',
            executable='patrol_token_dispenser_node',
            output='screen',
            name='interface_node',
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                 {"robot_id": "patrol_token_dispenser",
                  "team_members_file_name": "team_spec_inst_5r_20w.json",
                  "robots_spec_file_name": "robots_spec_inst_5r_20w_02.json",
                  "mission_spec_file_path": "mission_spec_inst_5r_20w_02.json"}
            ],
        ),
        included_launch
    ])

    return ld
