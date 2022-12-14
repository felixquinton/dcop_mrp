import itertools
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    #  TODO Have nb_robot set by an argument

    communication_range = LaunchConfiguration(
        'communication_range', default='1')

    scenario_path = LaunchConfiguration('scenario_path', default='1')

    scenario_id = LaunchConfiguration('scenario_id', default='1')

    auction_com_cost = LaunchConfiguration('auction_com_cost', default='1')

    method = LaunchConfiguration('method', default='SI')

    folder = LaunchConfiguration('folder', default='1')

    nb_robots = 5

    return LaunchDescription(list(itertools.chain.from_iterable([[
        Node(
            package='patrol_robot',
            namespace=f'robot_{i}',
            executable='patrol_robot_node',
            name=f'robot_{i}' + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
#            prefix="" if i == -1 else 'xterm -T ' + \
#            f'robot_{i}' + '_node' + ' -e',
            parameters=[
                {"robot_id": f'robot_{i}',
                 "team_members_file_name": "team_spec_inst_caylus_r5_w21.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_caylus_w21.json",
                 "communication_range": communication_range,
                 "scenario_path": scenario_path,
                 "scenario_id": scenario_id,
                 "auction_com_cost": auction_com_cost,
                 "method": method,
                 "folder": folder}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=f'robot_{i}',
            executable='patrol_comsim_node',
            name=f'robot_{i}' + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
#            prefix='xterm -T ' + f'robot_{i}' + '_com_node' + ' -e',
            parameters=[
                {"robot_id": f'robot_{i}',
                 "team_members_file_name": "team_spec_inst_caylus_r5_w21.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_caylus_w21.json",
                 "communication_range": communication_range,
                 "scenario_path": scenario_path,
                 "scenario_id": scenario_id,
                 "method": method,
                 "folder": folder}
            ]
        )] for i in range(1, int(nb_robots) + 1)]))
    )
