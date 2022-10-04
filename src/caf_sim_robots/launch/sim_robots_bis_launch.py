from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='caf_sim_robots',
            # namespace='turtlesim1',
            executable='sim_robots_node',
            name='sim_robots_bis_node'
        )
    ])