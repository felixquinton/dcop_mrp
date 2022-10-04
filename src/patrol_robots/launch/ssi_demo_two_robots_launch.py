from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_id_1 = 'robot_1'
    robot_id_2 = 'robot_2'
    #  TODO optimize creation of robots by calling for i in range
    # (1, nb_r) one_robot_launch

    return LaunchDescription([
        Node(
            package='patrol_robot',
            namespace=robot_id_1,
            executable='patrol_robot_node',
            name=robot_id_1 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_1 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_1,
                 "team_members_file_name": "ssi_demo_team_spec.json",
                 "robots_spec_file_name": "ssi_demo_robot.json",
                 "mission_spec_file_path": "ssi_demo_mission_spec.json"}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_1,
            executable='patrol_comsim_node',
            name=robot_id_1 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_1 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_1,
                 "team_members_file_name": "ssi_demo_team_spec.json",
                 "robots_spec_file_name": "ssi_demo_robot.json",
                 "mission_spec_file_path": "ssi_demo_mission_spec.json"}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_2,
            executable='patrol_robot_node',
            name=robot_id_2 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_2 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_2,
                 "team_members_file_name": "ssi_demo_team_spec.json",
                 "robots_spec_file_name": "ssi_demo_robot.json",
                 "mission_spec_file_path": "ssi_demo_mission_spec.json"}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_2,
            executable='patrol_comsim_node',
            name=robot_id_2 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_2 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_2,
                 "team_members_file_name": "ssi_demo_team_spec.json",
                 "robots_spec_file_name": "ssi_demo_robot.json",
                 "mission_spec_file_path": "ssi_demo_mission_spec.json"}
            ]
        ),
    ])
