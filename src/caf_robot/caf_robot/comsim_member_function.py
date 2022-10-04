"""Describes a node simulating the communication hazards occuring as messages
travel. The ComSim node takes a parameter robot_id that MUST correspond to a
unique Robot node's robot_id parameter to form a ComSim-Robot pair.
Once the ComSim received an announcement or bid message from the team, it will
continusously send it to its robot.
Once the ComSim received an announcement or bid message from its robot, it will
continusously send it to the team.
"""

import rclpy
import numpy as np
import json
import networkx as nx

from caf_messages.msg import (AnnouncementStamped, BidStamped, AuctionHeader,
                              BundleDescription, ItemDescription, Header,
                              ResultsStamped, AwardAcknowledgementStamped)

from std_msgs.msg import String

from caf_essential.base_node import BaseNode
from caf_essential.utils import utils

auctions_msg_types = [AnnouncementStamped, BidStamped, AuctionHeader,
                      BundleDescription, ItemDescription, Header,
                      ResultsStamped, AwardAcknowledgementStamped]


class ComSim(BaseNode):

    def __init__(self, node_name='comsim'):
        """Initialize the ComSim. The parameter robot_id is saved to form a
        pair with the ComSim's robot. Subscriptions and publishers are set up
        according to this parameter to ensure a one to one communication link
        between the ComSim and its robot.
        Parameters
        ----------
        Returns
        ----------
        """

        super().__init__(node_name)

        # Initialize node attributes with this robot information
        self.declare_parameter("robot_id")
        self.robot_id = self.get_parameter(
            "robot_id").get_parameter_value().string_value
        self.robot_spec_data = None
        if self.robot_id in self.robots_spec_data.keys():
            self.robot_spec_data = self.robots_spec_data[self.robot_id]
        else:
            self.get_logger().info('Cannot retrieve robot spec data')

        self.initialize_topics()

        utils.print_dict(self.node_listeners)
        utils.print_dict(self.node_publishers)

        # dict of received msg from the team
        self.received_msg_from_team = {}

        # message_id transferred to ignore
        self.transferred_msg_to_ignore = []

        # auctions to ignore due to com issues.

        # Test for lost auction com. The second auction is lost, so the
        # allocation should be r1 <- 2 ; r2 <- 4 & 9 ; r3 <- 5
        # self.lost_com_auctions = ["auction_robot_1_1"]
        self.com_loss_proba = 0
        self.com_graph = nx.empty_graph()
        self.relay_com = self.robot_spec_data["com_relay"]
        self.lost_com_auctions = []

        self.get_logger().info('Init OK.')

    def initialize_listeners(self, init_gt=True, init_auction=True):
        """Initialize all listeners of the robot"""
        # ------------------ Auctions topics
        print(init_auction)
        if init_auction:
            self.initialize_auctions_listeners()

        # Others topics, point-to-point communication
        # for robot_name in self.robots_list:
        #     self.node_listeners[topic_ns][robot_name] = {}
        if init_gt:
            self.initialize_ground_truth_listeners()

    def initialize_ground_truth_listeners(self):
        """
        Initialize the node's listeners. interface_node listens to the
        ground_truth topics from sim_robots for the robots' pose and energy
        status.
        """
        # Ground truth topics
        topic_ns = 'ground_truth'

        # Communication Network
        topic_info = 'com_network'
        msg_type = String
        listener_cb = self.update_com_network_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1000,
                                        ns_list=[[topic_ns]])

    def update_com_network_cb(self, msg, topic_name=None):
        self.com_graph = nx.readwrite.node_link_graph(json.loads(msg.data))
        self.get_logger().info("Updated com graph.")

    def initialize_auctions_listeners(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_auctions'
        topic_ns = 'auctions'
        # General announcement topic
        topic_info = 'announcement'
        msg_type = AnnouncementStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

        # General bid topic
        topic_info = 'bid'
        msg_type = BidStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

        # General results topic
        topic_info = 'results'
        msg_type = ResultsStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

        # General award acknowledgement topic
        topic_info = 'award_acknowledgement'
        msg_type = AwardAcknowledgementStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

    def initialize_publishers(self, init_auction=True):
        """Initialize all publishers of the robot"""
        # ------------------ Auctions topics
        if init_auction:
            self.initialize_auctions_publishers()

        # Others topics, point-to-point communication
        # for robot_name in self.robots_list:
        #     self.node_listeners[topic_ns][robot_name] = {}

        # ------------------ TODO Information, supervision... topics
        pass

    def initialize_auctions_publishers(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_auctions'
        topic_ns = 'auctions'
        # General announcement topic
        topic_info = 'announcement'
        msg_type = AnnouncementStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[comm_topic_ns], [topic_ns]])

        # General bid topic
        topic_info = 'bid'
        msg_type = BidStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[comm_topic_ns], [topic_ns]])

        # General result topic
        topic_info = 'results'
        msg_type = ResultsStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[comm_topic_ns], [topic_ns]])

        # General award acknowledgement topic
        topic_info = 'award_acknowledgement'
        msg_type = AwardAcknowledgementStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[comm_topic_ns], [topic_ns]])

    def transfer_msg_to_robot_cb(self, msg, topic_name=None,
                                 non_auction_msg_loss=False):
        # Check reception
        # On sender
        if msg.header.sender_id == self.robot_id:
            return

        to_robot = True

        if not self.relay_com and not (
                msg.header.sender_id.split('_')[-1],
                self.robot_id.split('_')[-1]) in self.com_graph.edges:
            to_robot = False
        elif self.relay_com:
            try:
                if not nx.algorithms.shortest_paths.generic.has_path(
                        self.com_graph,
                        msg.header.sender_id.split('_')[-1],
                        self.robot_id.split('_')[-1]):
                    to_robot = False
            except nx.exception.NodeNotFound:
                to_robot = False

        if isinstance(msg, AnnouncementStamped):
            if to_robot:
                if self.robot_spec_data["com_stable"] == "auction":
                    self.random_auction_com_lost(
                        msg.body_msg.auction_header.auction_id,
                        msg.header.sender_id)
                    if (msg.body_msg.auction_header.auction_id
                            in self.lost_com_auctions):
                        to_robot = False
                elif self.robot_spec_data["com_stable"] == "message":
                    if self.random_message_com_lost(msg.header.sender_id):
                        to_robot = False
        else:
            if type(msg) in auctions_msg_types:
                if self.robot_spec_data["com_stable"] == "auction":
                    if (msg.body_msg.auction_header.auction_id
                            in self.lost_com_auctions):
                        to_robot = False
                    else:
                        to_robot = True
                elif self.robot_spec_data["com_stable"] == "message":
                    if self.random_message_com_lost(msg.header.sender_id):
                        to_robot = False
            elif non_auction_msg_loss:
                if to_robot:
                    if self.random_message_com_lost(msg.header.sender_id):
                        to_robot = False
        if to_robot:

            # Transfer msg to robot node
            # self.get_logger().info(
            #     'Received a message from ' + msg.header.sender_id + '\n'
            #     + str(msg))
            self.received_msg_from_team[msg.header.message_id] = msg
            self.transferred_msg_to_ignore.append(msg.header.message_id)

            if type(msg) in auctions_msg_types:
                keys = topic_name.split('/')
                # If topic name has a '/' as first element, we need to remove
                # first key
                if keys[0] == '':
                    keys.remove('')
                keys[0] = 'auctions'
                # utils.print_dict(self.node_publishers)
                topic = utils.get_dict_value_list_keys(self.node_publishers,
                                                       keys)
                # print(topic)
                topic.publish(msg)
                keys.insert(0, self.robot_id)
                topic_name = utils.str_builder(keys, '/')
                self.get_logger().info(
                    'Message transferred to ' + topic_name)

    def transfer_msg_to_team_cb(self, msg, topic_name=None):
        """
        Callback called when a message is received from the robot node to
        transfer it to teammates
        :param msg: message to transfer
        :param topic_name:
        :return:
        """
        # Check if it is not a message sent by this own node
        if msg.header.message_id in self.transferred_msg_to_ignore:
            self.transferred_msg_to_ignore.remove(msg.header.message_id)
            return

        # self.get_logger().info(
        #     'Received a message from ' + msg.header.sender_id + '\n'
        #     + str(msg))

        if type(msg) in auctions_msg_types:
            keys = topic_name.split('/')
            keys[0] = 'comm_auctions'
            topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
            topic.publish(msg)
            topic_name = utils.str_builder(keys, '/')
            # self.get_logger().info(
            #     'Message transferred to ' + topic_name)

        else:
            # addressees = msg.header.addressees
            # if addressees:
            #     # TODO transfer msg to specific robots
            #     pass
            # else:
            #     # Broadcast
            #     pass
            pass

    def random_auction_com_lost(self, auction_id, sender_id):
        """
        Randomly determines if all messages regarding an auction are lost.
        :param auction_id: id of the auction.
        """
        robot_id = self.robot_id.split('_')[-1]
        teammate_id = sender_id.split('_')[-1]
        com_prob = 1
        com_path = nx.shortest_path(self.com_graph, source=teammate_id,
                                    target=robot_id)
        if len(com_path) > 1:
            for i in range(len(com_path)-1):
                com_prob *= self.com_graph[
                    com_path[i]][com_path[i+1]]["weight"]
        if np.random.rand() > com_prob:
            self.lost_com_auctions.append(auction_id)

    def random_message_com_lost(self, sender_id):
        """
        Randomly determines if all messages regarding an auction are lost.
        :param auction_id: id of the auction.
        :return: True if the message is lost, False otherwise.
        """
        robot_id = self.robot_id.split('_')[-1]
        teammate_id = sender_id.split('_')[-1]
        com_prob = 1
        com_path = nx.shortest_path(self.com_graph, source=teammate_id,
                                    target=robot_id)
        if len(com_path) > 1:
            for i in range(len(com_path)-1):
                com_prob *= self.com_graph[
                    com_path[i]][com_path[i+1]]["weight"]
        if np.random.rand() > com_prob:
            return True
        return False


def main(args=None):
    rclpy.init(args=args)

    comsim = ComSim()

    rclpy.spin(comsim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    comsim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
