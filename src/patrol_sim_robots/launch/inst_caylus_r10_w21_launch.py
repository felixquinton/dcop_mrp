import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    package_prefix = get_package_share_directory('patrol_robot')

    ld = LaunchDescription()
    communication_range = LaunchConfiguration('communication_range')
    scenario_path = LaunchConfiguration('scenario_path')
    scenario_id = LaunchConfiguration('scenario_id')
    nb_robots = LaunchConfiguration('nb_robots')
    auction_com_cost = LaunchConfiguration('auction_com_cost')
    method = LaunchConfiguration('method')
    folder = LaunchConfiguration('folder')

    ld = launch.LaunchDescription([
        DeclareLaunchArgument(
            'auction_com_cost', default_value="no_com",
            description="Type of auction cost in the auction bid valuation."),
        DeclareLaunchArgument(
            'scenario_id', default_value="1",
            description="Argument for scenario's (or instance's) id."),
        DeclareLaunchArgument(
            'communication_range', default_value="100",
            description="Argument for robot's communication range."),
        DeclareLaunchArgument(
            'folder', default_value="",
            description="Argument for the path of the logging folder."),
        DeclareLaunchArgument(
            'method', default_value="SI",
            description="Argument for MRTA solving method."),
        DeclareLaunchArgument(
            'nb_robots', default_value="5",
            description="Argument for number of robots."),
        DeclareLaunchArgument(
            'scenario_path', default_value="",
            description="Argument for the path to the scenario."),
        launch_ros.actions.Node(
            package='patrol_sim_robots',
            executable='patrol_sim_robots_node',
            output='screen',
            name='simulation_robots_node',
            # emulate_tty is necessary to use print
            emulate_tty=True,
            # prefix='xterm -e',
            parameters=[
                {"team_members_file_name": "team_spec_inst_caylus_r10_w21.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_caylus_w21.json",
                 "communication_range": communication_range,
                 "scenario_path": scenario_path,
                 "scenario_id": scenario_id,
                 "auction_com_cost": auction_com_cost,
                 "method": method,
                 "folder": folder}
            ],
        ),
#        launch_ros.actions.Node(
#        package='patrol_interface',
#        executable='patrol_interface_node',
#        output='screen',
#        name='interface_node',
#        emulate_tty=True,
#        # prefix='xterm -e',
#        parameters=[
#             {"robot_id": "interface_patrol",
#              "team_members_file_name": "team_spec_inst_caylus_r10_w21.json",
#              "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
#              "mission_spec_file_path": "mission_spec_inst_caylus_w21.json",
#              "communication_range": communication_range,
#              "scenario_path": scenario_path,
#              "method": method,
#              "scenario_id": scenario_id}
#        ],
#        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                package_prefix, '/ten_robots_inst_caylus_r10_21w_launch.py']),
            launch_arguments={
                'communication_range': communication_range,
                'nb_robots': nb_robots,
                "scenario_path": scenario_path,
                "scenario_id": scenario_id,
                "auction_com_cost": auction_com_cost,
                "method": method,
                "folder": folder}.items())
    ])

    return ld
