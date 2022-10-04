import rclpy
from patrol_essential.base_node import BaseNode
from patrol_essential.utils import utils
from std_msgs.msg import String


class TokenDispenser:
    """
    Class that manage all of the robot process
    TODO split planning, supervision, auction in others structures ?
    """

    def __init__(self, sim_node):
        self.sim_node = sim_node
        self.fifo_list = []

        self.initialize_listeners()
        self.initialize_publishers()

    def token_distributor(self, verbose=False):
        distrib_msg = String()
        if self.fifo_list:
            distrib_msg.data = self.fifo_list[0]
        self.sim_node.distrib_pub.publish(distrib_msg)
        if verbose:
            self.sim_node.get_logger().info(
                "Gave auctioneer token to robot "+str(distrib_msg))

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
        self.sim_node.create_subscription(
            msg_type, topic_ns+'/'+topic_info,
            listener_cb, 10)

        # Pass token
        topic_info = 'pass'
        msg_type = String
        listener_cb = self.receive_pass
        self.sim_node.create_subscription(
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
        self.sim_node.distrib_pub = self.sim_node.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def receive_request(self, msg):
        if msg.data not in self.fifo_list:
            self.fifo_list.append(msg.data)
        # self.sim_node.get_logger().error(f"receive_request {self.fifo_list}")

    def receive_pass(self, msg):
        if self.fifo_list and msg.data == self.fifo_list[0]:
            self.fifo_list = self.fifo_list[1:]
