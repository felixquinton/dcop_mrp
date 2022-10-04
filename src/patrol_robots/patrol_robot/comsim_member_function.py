"""Describes a node simulating the communication hazards occuring as messages
travel. The ComSim node takes a parameter robot_id that MUST correspond to a
unique Robot node's robot_id parameter to form a ComSim-Robot pair.
Once the ComSim received an announcement or bid message from the team, it will
continusously send it to its robot.
Once the ComSim received an announcement or bid message from its robot, it will
continusously send it to the team.
"""

import rclpy
import networkx as nx
import numpy as np
import csv
import os

from caf_robot.comsim_member_function import ComSim as BaseComSim
from caf_essential.utils import utils
from patrol_essential.utils import utils as patrol_utils
from caf_messages.msg import (AnnouncementStamped, BidStamped, AuctionHeader,
                              BundleDescription, ItemDescription, Header,
                              ResultsStamped, AwardAcknowledgementStamped)

from patrol_messages.msg import (LastVisitsStamped, TeamDestStamped,
                                 EditEdgeStamped, EditNodeStamped)

auctions_msg_types = [AnnouncementStamped, BidStamped, AuctionHeader,
                      BundleDescription, ItemDescription, Header,
                      ResultsStamped, AwardAcknowledgementStamped]

patrol_msg_types = [LastVisitsStamped, TeamDestStamped]
dynamic_events_msg_types = [EditEdgeStamped, EditNodeStamped]


class ComSim(BaseComSim):

    def __init__(self):
        """Initialize the ComSim. The parameter robot_id is saved to form a
        pair with the ComSim's robot. Subscriptions and publishers are set up
        according to this parameter to ensure a one to one communication link
        between the ComSim and its robot.
        Parameters
        ----------
        Returns
        ----------
        """

        super().__init__()
        self.robots_spec_data = patrol_utils.get_robots_spec_data(self)
        self.robot_spec_data = self.robots_spec_data[self.robot_id]
        self.relay_com = self.robot_spec_data["com_relay"]
        self.declare_parameter("scenario_id")
        self.folder_str = self.get_parameter("scenario_id").value

        self.declare_parameter("folder")
        self.first_path = self.get_parameter("folder").value + "/"

    def initialize_listeners(self):
        self.declare_parameter("method")
        self.method = self.get_parameter("method").value

        print(self.method, self.method in ["SI"])
        super().initialize_listeners(init_auction=(self.method in ["SI"]))
        self.initialize_patrol_listeners()
        self.initialize_dynamic_events_listeners()

    def initialize_patrol_listeners(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_patrol'
        topic_ns = 'patrol'
        # General announcement topic
        topic_info = 'last_visits'
        msg_type = LastVisitsStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners, listener_cb=listener_cb,
            queue_size=1000, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])
        topic_info = 'team_dest'
        msg_type = TeamDestStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners, listener_cb=listener_cb,
            queue_size=1000, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])

    def initialize_dynamic_events_listeners(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_events'
        topic_ns = 'event'
        # General edit edge topic
        topic_info = 'edit_edge'
        msg_type = EditEdgeStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners, listener_cb=listener_cb,
            queue_size=1000, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])
        # General edit node topic
        topic_info = 'edit_node'
        msg_type = EditNodeStamped
        listener_cb = self.transfer_msg_to_robot_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners, listener_cb=listener_cb,
            queue_size=1000, ns_list=[[comm_topic_ns]])
        listener_cb = self.transfer_msg_to_team_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])

    def initialize_publishers(self):
        super().initialize_publishers(init_auction=(self.method in ["SI"]))
        self.initialize_patrol_publishers()
        self.initialize_dynamic_events_publishers()

    def initialize_patrol_publishers(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_patrol'
        topic_ns = 'patrol'
        # General announcement topic
        topic_info = 'last_visits'
        msg_type = LastVisitsStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[comm_topic_ns], [topic_ns]])
        # General announcement topic
        topic_info = 'team_dest'
        msg_type = TeamDestStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[comm_topic_ns], [topic_ns]])

    def initialize_dynamic_events_publishers(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = '/comm_events'
        topic_ns = 'event'
        # General edit edge topic
        topic_info = 'edit_edge'
        msg_type = EditEdgeStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[comm_topic_ns], [topic_ns]])
        # General edit node topic
        topic_info = 'edit_node'
        msg_type = EditNodeStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[comm_topic_ns], [topic_ns]])

    def transfer_msg_to_team_cb(self, msg, topic_name=None):
        super().transfer_msg_to_team_cb(msg, topic_name)
        # Check if it is not a message sent by this own node
        if msg.header.message_id in self.transferred_msg_to_ignore:
            self.transferred_msg_to_ignore.remove(msg.header.message_id)
            return

        if type(msg) in patrol_msg_types:
            keys = topic_name.split('/')
            keys[0] = 'comm_patrol'
            topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
            topic.publish(msg)
            topic_name = utils.str_builder(keys, '/')
            # self.get_logger().info(
            #     'Message transferred to ' + topic_name)

        if type(msg) in dynamic_events_msg_types:
            # self.get_logger().info(f'Starting to transfer {msg} to team')
            keys = topic_name.split('/')
            keys[0] = 'comm_events'
            topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
            topic.publish(msg)
            topic_name = utils.str_builder(keys, '/')
            # self.get_logger().info(
            #     'Message transferred to ' + topic_name)

    def transfer_msg_to_robot_cb(self, msg, topic_name=None):
        # self.get_logger().warn(f'Starting to transfer {msg} to robot')
        super().transfer_msg_to_robot_cb(msg, topic_name)
        # if type(msg) in dynamic_events_msg_types:

        # Check reception
        # On sender
        if msg.header.sender_id == self.robot_id:
            # if type(msg) in dynamic_events_msg_types:
            return

        if not self.relay_com and not (
                msg.header.sender_id.split('_')[-1],
                self.robot_id.split('_')[-1]) in self.com_graph.edges:
            # if type(msg) in dynamic_events_msg_types:
            return
        elif self.relay_com:
            try:
                if not nx.algorithms.shortest_paths.generic.has_path(
                        self.com_graph,
                        msg.header.sender_id.split('_')[-1],
                        self.robot_id.split('_')[-1]):
                    return
            except nx.exception.NodeNotFound:
                return

        # Transfer msg to robot node
        # self.get_logger().info(
        #     'Received a message from ' + msg.header.sender_id + '\n'
        #     + str(msg))
        self.received_msg_from_team[msg.header.message_id] = msg
        self.transferred_msg_to_ignore.append(msg.header.message_id)

        lose_msg = self.random_message_com_lost(msg.header.sender_id)

        if (type(msg) in patrol_msg_types
                or type(msg) in dynamic_events_msg_types) and lose_msg:
            pass

        if type(msg) in patrol_msg_types and not lose_msg:
            keys = topic_name.split('/')
            # If topic name has a '/' as first element, we need to remove first
            # key
            if keys[0] == '':
                keys.remove('')
            keys[0] = 'patrol'
            # utils.print_dict(self.node_publishers)
            topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
            # print(topic)
            topic.publish(msg)
            keys.insert(0, self.robot_id)
            topic_name = utils.str_builder(keys, '/')
            # self.get_logger().info(
            #     'Message transferred to ' + topic_name)

        if type(msg) in dynamic_events_msg_types and not lose_msg:
            if type(msg) in dynamic_events_msg_types:
                # self.get_logger().warn(
                #     f'''I know this is a d_e msg and it must be sent to robot.
                #     {msg}''')
                pass
            keys = topic_name.split('/')
            # If topic name has a '/' as first element, we need to remove first
            # key
            if keys[0] == '':
                keys.remove('')
            keys[0] = 'event'
            # utils.print_dict(self.node_publishers)
            topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
            # print(topic)
            topic.publish(msg)
            keys.insert(0, self.robot_id)
            topic_name = utils.str_builder(keys, '/')
        # self.get_logger().info(
        #     'Message transferred to ' + topic_name)

    def random_auction_com_lost(self, auction_id, sender_id):
        """
        Randomly determines if all messages regarding an auction are lost.
        :param auction_id: id of the auction.
        """
        robot_id = self.robot_id.split('_')[-1]
        teammate_id = sender_id.split('_')[-1]
        com_prob = 0
        com_path = nx.shortest_path(self.com_graph, source=teammate_id,
                                    target=robot_id)
        if len(com_path) > 1:
            for i in range(len(com_path)-1):
                com_prob += self.com_graph[com_path[i]][
                    com_path[i+1]]["weight"]
        if np.random.rand() > com_prob:
            self.lost_com_auctions.append(auction_id)
            if not os.path.exists(self.first_path+str(self.folder_str)):
                os.makedirs(self.first_path+str(self.folder_str))
            log_file = (
                self.first_path+str(self.folder_str)
                + "/lost_msg_" + str(self.robot_id) + ".csv")
            with open(log_file, mode='a+') as outfile:
                csv_writer = csv.writer(outfile, delimiter=',',
                                        quotechar='"',
                                        quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([
                    self.robot_id,
                    str((
                        self.get_clock().now().to_msg().sec
                        + self.get_clock().now().to_msg().nanosec/1e9)),
                    "AnnouncementStamped"])
        else:
            pass


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
