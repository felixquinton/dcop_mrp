import rclpy
from rclpy.node import Node
import threading

# Messages for caf auction processes
from caf_messages.msg import (AnnouncementStamped, BidStamped,
                              AuctionHeader, BundleDescription,
                              ItemDescription, Header,
                              ResultsStamped, AwardAcknowledgementStamped)

# useful imports to call several tools
from caf_essential.utils import utils

# In ROS2 there isn't yet a callback_args to pass args to a listener_cb, the
# same can be done to ActionServer cbs partial func allow it
from functools import partial

auctions_msg_types = [AnnouncementStamped, BidStamped, AuctionHeader,
                      BundleDescription, ItemDescription, Header,
                      ResultsStamped, AwardAcknowledgementStamped]


class BundleHandlerBidder:
    """
    Class to handle bundle constructed by the bidder
    """
    # TODO move this class in another file in case of a splitting of auctions processes

    def __init__(self, bid_msg):
        self.bid_msg = bid_msg

        # extract information from bid_msg and bundle_description for convenience
        bundle_description = bid_msg.bundle_description
        self.bundle_id = bundle_description.bundle_id
        self.bundle_description = bundle_description
        self.bid = bid_msg.bid

        # Indict if tasks have been extracted from bundle
        # self.are_tasks_extracted = False


class ClientHandler:
    """
    This class manages an ActionClient and its related information
    """

    def __init__(self, action_name, action_type, client_name, client):
        self.action_name = action_name  # string (ex: "GoTo")
        self.action_type = action_type  # ROS2 Action type (ex: GoTo)
        # string (ex: "/robot_1/fl_actions/GoTo")
        self.client_name = client_name
        self.client = client  # Created ActionClient

    def print_info(self):
        """
        Print ClientHandler information
        """
        print('ClientHandler info:\n',
              'action_name : ', self.action_name, '\n',
              'action_type : ', self.action_type, '\n',
              'client_name : ', self.client_name, '\n',
              'client : ', self.client
              )


class ServerHandler:
    """
    This class manages an ActionServer and its related information
    """

    def __init__(self, action_name, action_type, server_name, server, cb_dict):
        self.action_name = action_name  # string (ex: "GoTo")
        self.action_type = action_type  # ROS2 Action type (ex: GoTo)
        # string (ex: "/robot_1/fl_actions/GoTo")
        self.server_name = server_name
        self.server = server  # Created ActionServer

        # Dict of callbacks to use with this action server
        self.cb_dict = cb_dict

        # cb called to execute a goal
        self.execute_callback = cb_dict['execute_callback']
        # a group cb allows cb from the same group to be executed in concurrency
        self.callback_group = cb_dict['callback_group']
        # cb called when a goal is received
        self.goal_callback = cb_dict['goal_callback']
        # cb called when a cancel order is received
        self.cancel_callback = cb_dict['cancel_callback']

    def print_info(self):
        print('ServerHandler info:\n',
              'action_name : ', self.action_name, '\n',
              'action_type : ', self.action_type, '\n',
              'server_name : ', self.server_name, '\n',
              'server : ', self.server, '\n',
              'execute_callback : ', self.execute_callback, '\n',
              'callback_group : ', self.callback_group, '\n',
              'goal_callback : ', self.goal_callback, '\n',
              'cancel_callback : ', self.cancel_callback
              )


class BaseNode(Node):
    """
    Mother class for all CAF nodes.
    This an important class. Inside we retrieve statics mission information
    useful for each nodes but we also create all ROS2 necessary elements.
    By example we define functions for publishers and subscribers.
    """

    def __init__(self, node_name='base_node', positions=None):
        super().__init__(node_name)

        self.get_logger().info('Initializing node...')

        # mutex to modify robot attributes
        self.lock = threading.RLock()

        # ------------- Get team members information --------------------------
        # Retrieve team members information
        self.team_members_file_path = None
        self.team_members_data = None
        self.team_members_data = utils.get_team_members_data(self)
        self.robots_list = self.team_members_data['team_members_list']

        # ------- Retrieve robots spec information to use ---------------------
        self.robots_spec_file_name = None
        self.robots_spec_data = None
        self.robots_spec_data = utils.get_robots_spec_data(self)

        # ----------- Retrieve mission spec information to use ----------------
        self.mission_spec_file_path = None
        self.mission_spec_data = None
        self.mission_spec_data = utils.get_mission_spec_data(self)

        # --------- Initialize listeners and publishers attributes ------------
        self.node_listeners = {}
        self.node_publishers = {}
        self.msg_id_cpt = 0

    def initialize_topics(self):
        """
        This function called functions to initialized nodes listeners and publishers.
        Usually called after robots spec was retrieved.
        """
        self.initialize_listeners()
        self.initialize_publishers()

    def initialize_listeners(self):
        """
        Function to initialize node listeners
        NEEDS to be surcharged
        """
        pass

    def initialize_publishers(self):
        """
        Function to initialize node publishers
        NEEDS to be surcharged
        """
        pass

    def create_and_store_listeners(self, topic_info, msg_type, storage_dict,
                                   listener_cb=None, queue_size=10,
                                   ns_list=None):
        """
        Init and store listeners in a specified dict
        If specified, will create a listeners for each topic_ns_list
        :param topic_info: topic name, without namespace
        :param msg_type: message type of the topic
        :param storage_dict: dict to store publishers
        :param listener_cb: Callback to use
        :param queue_size: queue size for publishers
        :param ns_list: namespace for topic name and storage dict
        """
        if not topic_info or not msg_type:
            # TODO throw exception
            self.get_logger().info(
                'ERROR: create_and_store_listeners, not a valid topic_info ou msg_type')
            return

        if not listener_cb:
            listener_cb = self.default_listener_cb

        if not ns_list:
            topic_name = topic_info
            storage_dict[topic_name] = self.create_subscription(
                msg_type, topic_name,
                partial(listener_cb, topic_name=topic_name), queue_size)
        else:
            for ns in ns_list:
                _list = list(ns) + [topic_info]
                topic_name = utils.str_builder(_list, '/')
                keys = utils.replace_character_from_list(_list, '/')
                d = utils.get_dict_value_list_keys(
                    storage_dict, keys, init=True,
                    return_element='second_to_last')
                d[topic_info] = self.create_subscription(
                    msg_type, topic_name,
                    partial(listener_cb, topic_name=topic_name), queue_size)

    def default_listener_cb(self, msg, topic_name=None):
        self.get_logger().info(
            "DEFAULT LISTENER CB : Retrieved a message from topic : " + topic_name)
        pass

    def create_and_store_publishers(self, topic_info, msg_type, storage_dict,
                                    queue_size=10, ns_list=None):
        """
        Init and store publishers in a specified dict
        If specified, will create a publisher for each topic_ns_list
        :param topic_info: topic name, without namespace
        :param msg_type: message type of the topic
        :param storage_dict: dict to store publishers
        :param queue_size: queue size for publishers
        :param ns_list: namespace for topic name and storage dict
        """
        if not topic_info or not msg_type:
            # TODO throw exception
            self.get_logger().info(
                'ERROR: create_and_store_publishers, not a valid topic_info ou msg_type')
            return

        if not ns_list:
            topic_name = topic_info
            storage_dict[topic_name] = self.create_publisher(
                msg_type, topic_name, queue_size)
        else:
            for ns in ns_list:
                _list = list(ns) + [topic_info]
                topic_name = utils.str_builder(_list, '/')
                keys = utils.replace_character_from_list(_list, '/')
                d = utils.get_dict_value_list_keys(
                    storage_dict, keys, init=True,
                    return_element='second_to_last')
                d[topic_info] = self.create_publisher(
                    msg_type, topic_name, queue_size)

    def publish_msg(self, msg, topic_keys):
        """
        This function publish a message on a topic.
        :param msg: message to publish
        :param topic_keys: keys to retrieve publisher in node publishers dict
        :return:
        """
        topic = utils.get_dict_value_list_keys(
            self.node_publishers, topic_keys)
        if not topic:
            self.get_logger().error("Impossible to publish message")
            return
        else:
            topic.publish(msg)

    def send_msg_to_teammates(self, msg, **kwargs):
        """
        Get topic and send msg to teammates
        :param msg: msg to send
        """
        # Currently only publish auctions messages, needs to be extended to handle by example supervision messages
        if type(msg) in auctions_msg_types:
            self.send_auction_msg(msg, **kwargs)
        else:
            self.send_specific_msg(msg, **kwargs)

    def send_auction_msg(self, msg,
                         announcement_type=AnnouncementStamped,
                         bid_type=BidStamped,
                         result_type=ResultsStamped,
                         award_type=AwardAcknowledgementStamped):
        """
        Send message related to auction process
        :param msg: auction message to send
        :param announcement_type: type of the annoucement stamped messages
        :param bid_type: type of the bid stamped messages
        :param result_type: type of the results stamped messages
        :param award_type: type of the award acknowledgement stamped messages
        """
        keys = ['auctions']
        if isinstance(msg, announcement_type):
            keys.append('announcement')
        elif isinstance(msg, bid_type):
            keys.append('bid')
        elif isinstance(msg, result_type):
            keys.append('results')
        elif isinstance(msg, award_type):
            keys.append('award_acknowledgement')
        else:
            self.get_logger().error(
                'Impossible to send msg')
        # topic = utils.get_dict_value_list_keys(self.node_publishers, keys)
        # topic.publish(msg)
        print(msg, keys)
        self.publish_msg(msg, keys)
        topic_name = utils.str_builder(keys, '/')
        self.get_logger().info(
            'Message transferred to ' + topic_name)

    def send_specific_msg(self, msg):
        """
        Empty method to handle mission specific messages.
        :param msg: auction message to send
        """
        pass

    def check_initialization(self):
        """
        This function verify that node initialisation was a success.
        """
        # Currently only print a basic message. Needs to be extended in order to improve CAF reliability
        self.get_logger().info("The node was initialized without problems")


def main(args=None):
    rclpy.init(args=args)

    base_node = BaseNode()

    rclpy.spin(base_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
