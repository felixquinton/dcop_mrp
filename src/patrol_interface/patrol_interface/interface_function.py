import rclpy
import json
import os
import random
import numpy as np
import networkx as nx
from ament_index_python.packages import get_package_share_directory
from patrol_essential.auction import navigation_graph as nv
from patrol_messages.msg import (LastVisitsStamped, EditEdge)
from caf_interface.interface_function import Interface as BaseInterface
from patrol_essential.utils import utils
from std_msgs.msg import Float64
from patrol_interface.auction_display import AuctionDisplay
from caf_messages.msg import (AnnouncementStamped,
                              AwardAcknowledgementStamped,
                              BidStamped,
                              ResultsStamped)
from patrol_interface.interface_v2 import PatrolInterfaceV2


class Interface(BaseInterface):
    """
    ROS2 node receiving data from ground_truth topics. Transfers
    this data to patrol_interface MissionDisplay for displaying.
    """

    def __init__(self):
        super().__init__()

    def _initialize_interface(self):

        package_prefix = get_package_share_directory('patrol_mission_spec')
        self.get_logger().info(f"HELLOO  {package_prefix}")

        self.declare_parameter("communication_range", 100)
        robots_com_range = self.get_parameter("communication_range").value

        self.declare_parameter("method", "SI")
        self.method = self.get_parameter("method").value

        self.declare_parameter("scenario_path", "")
        scenario_path = self.get_parameter("scenario_path").value
        print(f"This is scenario_path from interface {scenario_path}")

        self.team_members_data = utils.get_team_members_data(self)
        self.robots_spec_data = utils.get_robots_spec_data(self)

        self.robot_colors = self._init_colors()

        if scenario_path:
            data_path = os.path.join(
                get_package_share_directory('patrol_mission_spec'), 'data',
                scenario_path)
            with open(data_path) as datafile:
                data = json.loads(json.load(datafile))

            self.mission_spec_data = utils.import_generated_nav_graph(data)

            self.robots_com_range = {
                r_id: robots_com_range for r_id in self.robots_spec_data}
        else:
            self.mission_spec_data = utils.get_mission_spec_data(self)["graph"]
            self.robots_com_range = {
                r_id: self.robots_spec_data[r_id]["com_range"]
                for r_id in self.robots_spec_data}

        self.simulation_time = 0
        # Stores the items to display
        self.items = {}

        self.initialize_display()

        self.initialize_topics()

        self.nav_graph = nv.navGraph(self.mission_spec_data)

        self.last_visits = {w: 0 for w in self.nav_graph.nodes}

        self.update_idleness = True  # Update idleness every two cb calls.

        self.auction_display = AuctionDisplay()

    def initialize_display(self):
        """
        Initialize the MissionDisplay object with the navigation graph.
        """
        self.interface_v2 = PatrolInterfaceV2(self.robots_spec_data)
        self.interface_v2.mission_display.init_display_from_spec_data(
            self.mission_spec_data, self.robots_spec_data)

    def update_pose_cb(self, msg, topic_name=None):
        """
        Callback called when receiving a pose message. Upon reception, calls
        methods from MissionDisplay to update robot's position on the display.
        :param msg: ROS2 msg of type Pose, the message stating updated pose
          data.
        :param topic_name:
        """
        robot_id = topic_name.split('/')[0]
        if robot_id not in self.items:
            self.items[robot_id] = \
                self.interface_v2.mission_display.create_robot(
                    robot_id, self.robots_com_range[robot_id],
                    robot_colors=self.robot_colors)
        self.interface_v2.mission_display.update_positions(
            self.items[robot_id],
            (msg.position.x,
             self.interface_v2.mission_display.winfo_height() - msg.position.y)
        )
        self.interface_v2.mission_display.update_display()

    def update_timer_cb(self):
        self.interface_v2.mission_display.update_display()
        if self.update_idleness:
            updated_idle_array = np.array(list(self.last_visits.values()))
            self.interface_v2.idleness_display.update_data_mean_and_max(
                self.simulation_time,
                np.mean(self.simulation_time-updated_idle_array),
                np.max(self.simulation_time-updated_idle_array))
        self.update_idleness = not self.update_idleness

    def initialize_listeners(self):
        super().initialize_listeners()

        # Ground truth topics
        topic_ns = 'ground_truth'
        ns_list = [[self.robots_list[0], topic_ns]]

        # Simulation time
        topic_info = 'time'
        msg_type = Float64
        listener_cb = self.update_time_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=ns_list)

        self.initialize_idleness_listener()

        if self.method in ["SI"]:
            self.initialize_auction_listener()

        self.initialize_event_listener()

    def update_time_cb(self, msg, topic_name=None):
        self.simulation_time = msg.data

    def initialize_event_listener(self):
        # idleness topic
        topic_ns = 'event'
        ns_list = []
        for robot_name in self.robots_list:
            ns_list.append([robot_name, topic_ns])

        # Pose
        topic_info = 'edit_edge'
        msg_type = EditEdge
        listener_cb = self.update_edges_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=ns_list)

    def initialize_idleness_listener(self):
        # idleness topic
        topic_ns = 'patrol'
        ns_list = []
        for robot_name in self.robots_list:
            ns_list.append([robot_name, topic_ns])

        # Pose
        topic_info = 'last_visits'
        msg_type = LastVisitsStamped
        listener_cb = self.update_idle_cb
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=ns_list)

    def initialize_auction_listener(self):
        # idleness topic
        comm_topic_ns = '/comm_auctions'
        # announcement
        topic_info = 'announcement'
        msg_type = AnnouncementStamped
        listener_cb = self.receive_announcement_msg
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[comm_topic_ns]])
        # bid
        topic_info = 'bid'
        msg_type = BidStamped
        listener_cb = self.receive_bid_msg
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=[[comm_topic_ns]])
        # results
        topic_info = 'results'
        msg_type = ResultsStamped
        listener_cb = self.receive_result_msg
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=[[comm_topic_ns]])
        # results
        topic_info = 'award_acknowledgement'
        msg_type = AwardAcknowledgementStamped
        listener_cb = self.receive_acknowledgement_msg
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1, ns_list=[[comm_topic_ns]])

    def receive_announcement_msg(self, msg, topic_name=None):
        text = "Robot " + msg.header.sender_id + " "
        content = msg.body_msg
        text += "emitted an Announcement message for item "
        text += content.item_list[0].item_id
        self.auction_display.update_auction_text(text)

    def receive_bid_msg(self, msg, topic_name=None):
        text = "Robot " + msg.header.sender_id + " "
        content = msg.body_msg
        text += "emitted a Bid message for item "
        text += content.bundle_description.item_bundle[0].item_id
        text += " with value " + str(round(content.bid[0], 3))
        text += " and bundle id " + content.bundle_description.bundle_id
        self.auction_display.update_auction_text(text)

    def receive_result_msg(self, msg, topic_name=None):
        text = "Robot " + msg.header.sender_id + " "
        content = msg.body_msg
        text += "emitted a Results message for item "
        try:
            text += content.bundle_descriptions[0].item_bundle[0].item_id
            text += ". Winner is robot " + content.award_list[0].bidder_id
        except IndexError:
            text += ". There was no valid bids."
        self.auction_display.update_auction_text(text)

    def receive_acknowledgement_msg(self, msg, topic_name=None):
        text = "Robot " + msg.header.sender_id + " "
        content = msg.body_msg
        text += "emitted a Acknowledgement message for bundle "
        text += content.bundle_id
        self.auction_display.update_auction_text(text)

    def update_idle_cb(self, msg, topic_name=None):
        """
        Callback called when receiving a pose message. Upon reception, calls
        methods from MissionDisplay to update robot's position on the display.
        :param msg: ROS2 msg of type Pose, the message stating updated pose
          data.
        :param topic_name:
        """
        idle_data = json.loads(msg.body_msg.last_visits)
        # print(idle_data, self.last_visits)
        for waypoint in self.last_visits:
            if idle_data[str(waypoint)] > self.last_visits[waypoint]:
                self.last_visits[waypoint] = idle_data[str(waypoint)]

    def update_edges_cb(self, msg, topic_name=None):
        """
        Callback called when receiving an edge event message. Upon reception,
        adds an argument to the edge in the nav_graph to indicate it's modified
        :param msg: ROS2 msg of type EditEdge.
        :param topic_name:
        """
        self.get_logger().error("In update edges cb")
        source, target = msg.source, msg.target
        p1 = self.nav_graph.convert_node_into_pos(source)
        p2 = self.nav_graph.convert_node_into_pos(target)
        self.nav_graph[source][target]["forbidden_type"] = msg.robot_types
        self.get_logger().error("Called edit edges")
        self.display.edit_edge(p1, p2, source, target, msg.robot_types)

    def update_ener_cb(self, msg, topic_name=None):
        pass

    def update_com_network_cb(self, msg, topic_name=None):
        com_graph = nx.readwrite.node_link_graph(json.loads(msg.data))
        self.interface_v2.mission_display.update_com_network(com_graph)

    def _init_colors(self):
        """
        Initialize robots' display colors
        """
        return {r_id: "#" + ''.join([random.choice('0123456789ABCDEF')
                                     for i in range(6)])
                for r_id in self.robots_spec_data.keys()}


def main(args=None):
    rclpy.init(args=args)

    interface_node = Interface()

    rclpy.spin(interface_node)

    interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
