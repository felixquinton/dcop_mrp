"""
Deprecated module. Now included in sim_robots as token_dispenser_class.
"""

import rclpy
from patrol_essential.base_node import BaseNode
from patrol_essential.utils import utils
from std_msgs.msg import String


class TokenDispenser(BaseNode):
    """
    Class that manage all of the robot process
    TODO split planning, supervision, auction in others structures ?
    """

    def __init__(self, node_name='token_dispenser'):
        super().__init__(node_name)

        self.team_members_data = utils.get_team_members_data(self)
        self.robots_spec_data = utils.get_robots_spec_data(self)
        self.mission_spec_data = utils.get_mission_spec_data(self)

        self.fifo_list = []

        self.initialize_topics()

        self.distribute_token = self.create_timer(.3, self._token_distributor)

    def _token_distributor(self):
        distrib_msg = String()
        if self.fifo_list:
            distrib_msg.data = self.fifo_list[0]
        self.distrib_pub.publish(distrib_msg)
        print(distrib_msg)

    def initialize_listeners(self):
        """
        Initialize all listeners of the robot
        """
        self.initialize_token_request_listeners()

    def initialize_token_request_listeners(self):
        """
        Initialize the node's listeners. interface_node listens to the
        ground_truth topics from sim_robots for the robots' pose and energy
        status.
        """
        # Token topic
        topic_ns = 'token'

        # Resquests
        topic_info = 'request'
        msg_type = String
        listener_cb = self.receive_request
        self.create_subscription(
            msg_type, topic_ns+'/'+topic_info,
            listener_cb, 10)

        # Pass token
        topic_info = 'pass'
        msg_type = String
        listener_cb = self.receive_pass
        self.create_subscription(
            msg_type, topic_ns+'/'+topic_info,
            listener_cb, 10)

    def initialize_publishers(self):
        """
        Initialize all publishers of the node
        """
        self.initialize_token_publishers()

    def initialize_token_publishers(self):
        topic_ns = 'token'
        # General announcement topic
        topic_info = 'distribution'
        msg_type = String
        self.distrib_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def receive_request(self, msg):
        if msg.data not in self.fifo_list:
            self.fifo_list.append(msg.data)

    def receive_pass(self, msg):
        if self.fifo_list and msg.data == self.fifo_list[0]:
            self.fifo_list = self.fifo_list[1:]


def main(args=None):
    rclpy.init(args=args)

    token_dispenser_node = TokenDispenser()

    rclpy.spin(token_dispenser_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    token_dispenser_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
