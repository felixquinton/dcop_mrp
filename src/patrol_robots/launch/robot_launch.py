from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    robot_id = 'robot_1'
    return LaunchDescription([
        Node(
            package='patrol_robot',
            namespace=robot_id,
            executable='patrol_robot_node',
            name=robot_id + '_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id + '_node' + ' -e',
            parameters=[
                {"robot_id": robot_id}
            ]
        ),
        Node(
            package='patrol_robot',
            namespace=robot_id,
            executable='patrol_comsim_node',
            name=robot_id + '_com_node',
            output="screen",
            emulate_tty=True,
            # to open the node in another term
            prefix='xterm -T ' + robot_id + '_com_node' + ' -e',
            parameters=[
                {"robot_id": robot_id}
            ]
        )
        ]
    )
