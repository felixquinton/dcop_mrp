from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_id_1 = 'robot_1'
    robot_id_2 = 'robot_2'
    robot_id_3 = 'robot_3'
    robot_id_4 = 'robot_4'
    robot_id_5 = 'robot_5'
    #  TODO optimize creation of robots by calling for i in range (1, nb_r)
    # one_robot_launch

    ind_file = open("./mission_logs/ind.txt", "r")
    print(ind_file)
    inst_id = ind_file.read()

    return LaunchDescription([
        Node(
            package='patrol_robot',
            namespace=robot_id_1,
            executable='patrol_robot_node',
            name=robot_id_1 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            # prefix='xterm -T ' + robot_id_1 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_1,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
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
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
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
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
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
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_3,
            executable='patrol_robot_node',
            name=robot_id_3 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_3 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_3,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_3,
            executable='patrol_comsim_node',
            name=robot_id_3 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_3 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_3,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_4,
            executable='patrol_robot_node',
            name=robot_id_4 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_4 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_4,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_4,
            executable='patrol_comsim_node',
            name=robot_id_4 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_4 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_4,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_5,
            executable='patrol_robot_node',
            name=robot_id_5 + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_5 + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_5,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id_5,
            executable='patrol_comsim_node',
            name=robot_id_5 + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id_5 + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id_5,
                 "team_members_file_name": "team_spec_inst_5r_20w.json",
                 "robots_spec_file_name": "robots_spec_inst_5r_20w_01.json",
                 "mission_spec_file_path": "mission_spec_inst_5r_20w_01.json",
                 "inst_id": inst_id}
            ]
        ),
    ])
