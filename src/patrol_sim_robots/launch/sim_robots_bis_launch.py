from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_sim_robots',
            # namespace='turtlesim1',
            executable='patrol_sim_robots_node',
            name='sim_robots_bis_node'
        )
    ])
