"""Describes a robot node. The Robot node takes a parameter robot_id that MUST
correspond to a unique ComSim node's robot_id parameter to form a ComSim-Robot
pair. The Robot node randomly generate announcement messages that it sends to
its ComSim. It is also able to receive annoucement messages and bid messages
from the ComSim. While bid messages are ignored in the present version,
annoucement messages trigger a bid message that is sent to the ComSim.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
import time
from caf_essential.auction import single_item_auction as si
from caf_essential.auction import sequential_single_item_auction as ssi

# Messages for caf auction processes
from caf_messages.msg import (AnnouncementStamped,
                              AwardAcknowledgementStamped,
                              BidStamped,
                              ItemDescription,
                              ResultsStamped)
from geometry_msgs.msg import Pose

from caf_essential.base_node import (BaseNode, ClientHandler)
from caf_essential.utils import utils, get_actions
from caf_essential.auction import navigation_graph as nv

# caf basic actions for example, replace by your use case
from caf_messages.action import GoTo, AbstractAction

from std_msgs.msg import Float64

# TODO adapt to avoid dependency
from caf_essential.planning import simple_sequential_planning
import traceback
import networkx as nx


class Robot(BaseNode):
    """
    Class that manage all of the robot process
    TODO split planning, supervision, auction in others structures ?
    """

    def __init__(self, node_name='robot_node'):
        """Initialize the Robot. The parameter robot_id is saved to form a pair
        with the robot's ComSim. Subscriptions and publishers are set up
        according to this parameter to ensure a one to one communication link
        between the robot and its ComSim.
        Parameters
        ----------
        Returns
        ----------
        """
        # ------------------------------ INIT BASE NODE -----------------------
        super().__init__(node_name)
        # ------------------------------ END BASE NODE INIT--------------------

        # ------------------------------ INIT ROBOT SPEC ATTRIBUTE ------------
        # Initialize node attributes with this robot information
        self.declare_parameter("robot_id")
        self.robot_id = self.get_parameter(
            "robot_id").get_parameter_value().string_value

        self.declare_parameter("method", "")
        self.method = self.get_parameter("method").value
        if self.method not in ["SI", "DCOP"]:
            self.get_logger().warn(
                f"Unsupported solving method: {self.method}")
        self.get_logger().info(
            f"Setting up robot with solving method {self.method}")

        self.robot_spec_data = None

        if self.robot_id in self.robots_spec_data.keys():
            self.robot_spec_data = self.robots_spec_data[self.robot_id]
        else:
            self.get_logger().info('Cannot retrieve robot spec data')

        try:
            self.fl_actions_package_name = self.robot_spec_data[
                'fl_actions_package_name']
        except KeyError:
            self.fl_actions_package_name = "caf_messages"

        print(self.mission_spec_data)
        self.nav_graph = nv.navGraph(self.mission_spec_data)
        self.shortest_paths = dict(nx.shortest_path(
            self.nav_graph, weight='weight'))
        self.path_lenghts = dict(nx.shortest_path_length(
            self.nav_graph, weight='weight'))
        self.position = (0., 0.)
        self.energy_cons = self.robot_spec_data["robot_energy_cons"]
        self.energy = 0.

        # ----------------------- INIT ROBOT SPEC ATTRIBUTE INIT---------------
        # Define a callbacks group allowed to be concurrent
        self.callback_group = ReentrantCallbackGroup()

        # -------------------------INIT AUCTIONS MANAGEMENT -------------------
        # If the robot start an auction, auction data are stored here
        self.passed_auctions_auctioneer = []
        # auctions from bidder side
        self.current_auction_bidder = None
        self.passed_auctions_bidder = {}

        # Award received
        # obtained_bundles is a dict of BundleHandlerBidder with bundle_id as key
        self.obtained_bundles = {}
        # self.plan = []
        self.busy = False

        self._set_auction_scheme()

        # ------------------ END AUCTIONS MANAGEMENT INIT ---------------------

        # ----- Initialize all ROS necessary bridges between nodes and thread
        # management tools -----
        self.initialize_topics()

        # Initialize action clients of the node
        self.node_actions_clients = {}
        self.initialize_actions_clients()
        # ----------------------------- END ROS INIT --------------------------

        # -------------- INIT PLANNING AND EXECUTION ATTRIBUTES ---------------
        # Timer to plan obtained bundles
        self.timer_plan = None

        # Unique goal id for sent goal (ros2 actions) to functional layer
        # TODO replace by a UUID
        self.goal_id = 1

        # Init a Plan object of the robot to manages items and Actions
        self.robot_plan = simple_sequential_planning.SimpleSequentialPlan(self)

        # stores received goals result from functional layer
        self.goals_results = []
        self.goals_handles = []
        # ------------- END PLANNING AND EXECUTION ATTRIBUTES INIT ------------

        # ------------- RUN AND DEBUG METHODS TO TEST ARCHITECTURE ------------
        if self.method == "SI":
            self.test_auctions_timer = self.create_timer(
                3.0, self.test_auctions_v2, callback_group=self.callback_group)

        # ------------- LOG INIT INFORMATION ---------------------------------
        self.check_initialization()

    def _set_auction_scheme(self):
        """
        Allows to set an auction scheme from a sub-class that extends this one.
        """
        self.auction_scheme = si.SingleItemAuction(self)  # TODO use a param
        # self.auction_scheme = ssi.SSIAuction(self)  # TODO use a param

    def test_auctions_v2(self):
        """
        Improved auction testing that assign a GoTo to each waypoint.
        """
        self.test_auctions_timer.cancel()
        # Choose robot_1 as first auctioneer
        if self.robot_id == 'robot_1':
            self.auctioneer_auction_to_start = []
            positions = nx.get_node_attributes(self.nav_graph, "pos")
            for waypoint in self.nav_graph.nodes:
                # Define item to sell
                item1 = ItemDescription()
                item1.item_id = 'item_'+str(waypoint)
                item_type = 'GoTo'
                if item_type == 'GoTo':
                    item1.item_name = 'goto_start_pos'
                    item1.item_type = 'GoTo'
                    item1.item_data = ('{"target_position": {"x": '
                                       + str(float(positions[waypoint][0]))
                                       + ', "y": '
                                       + str(float(positions[waypoint][1]))
                                       + ', "z": 0.0}}')
                else:
                    item1.item_name = 'abs'
                    item1.item_type = 'AbstractAction'
                    item1.item_data = '{"target_duration": 4.5}'
                # Define second item to sell
                self.auctioneer_auction_to_start.append(item1)
            self.auction_cpt = 0
            self.auction_scheme.send_announcement()

    def test_auctions(self):
        """
        Temporary function to init an auction process and test architecture
        """
        # Choose robot_1 as first auctioneer
        if self.robot_id == 'robot_1':
            self.auctioneer_auction_to_start = []
            # Define first item to sell
            item1 = ItemDescription()
            item1.item_id = 'item_1'
            item_type = 'GoTo'
            if item_type == 'GoTo':
                item1.item_name = 'goto_start_pos'
                item1.item_type = 'GoTo'
                item1.item_data = '{"target_position": {"x": 585.0, "y": 323.0, "z": 0.0}}'
            else:
                item1.item_name = 'abs'
                item1.item_type = 'AbstractAction'
                item1.item_data = '{"target_duration": 4.5}'
            # Define second item to sell
            self.auctioneer_auction_to_start.append(item1)
            item2 = ItemDescription()
            item2.item_id = 'item_2'
            item2_type = 'GoTo'
            if item2_type == 'GoTo':
                item2.item_name = 'goto_start_pos'
                item2.item_type = 'GoTo'
                item2.item_data = '{"target_position": {"x": 102.0, "y": 357.0, "z": 0.0}}'
            else:
                item2.item_name = 'abs'
                item2.item_type = 'AbstractAction'
                item2.item_data = '{"target_duration": 4.5}'
            self.auctioneer_auction_to_start.append(item2)
            # Define third item to sell
            item3 = ItemDescription()
            item3.item_id = 'item_3'
            item3_type = 'GoTo'
            if item3_type == 'GoTo':
                item3.item_name = 'goto_start_pos'
                item3.item_type = 'GoTo'
                item3.item_data = '{"target_position": {"x": 250.0, "y": 120.0, "z": 0.0}}'
            else:
                item3.item_name = 'abs'
                item3.item_type = 'AbstractAction'
                item3.item_data = '{"target_duration": 4.5}'
            self.auctioneer_auction_to_start.append(item3)
            item4 = ItemDescription()
            item4.item_id = 'item_3'
            item4_type = 'GoTo'
            if item4_type == 'GoTo':
                item4.item_name = 'goto_start_pos'
                item4.item_type = 'GoTo'
                item4.item_data = '{"target_position": {"x": 467.0, "y": 289.0, "z": 0.0}}'
            else:
                item4.item_name = 'abs'
                item4.item_type = 'AbstractAction'
                item4.item_data = '{"target_duration": 4.5}'
            self.auctioneer_auction_to_start.append(item4)
            self.auction_cpt = 0
            self.auction_scheme.send_announcement()

    # -------------------INITIALIZE CALLBACKS FOR CLIENTS ---------------------
    def get_action_client_cb(self, action_type):
        """
        Return a dict with the callback to use to init this type of ActionClient
        Surcharge this function to adapt CAF to your use case
        TODO currently its only useful for callback_group, delete this method ?
        :param action_type: Action type of the ActionClient
        :return:
        """
        # Use CAF default CB
        if action_type == GoTo:
            cb_dict = {
                'goal_response_callback': self.default_goal_response_callback,
                'callback_group': self.callback_group,
                'feedback_callback': self.default_feedback_callback,
                'result_callback': self.default_result_callback}
        elif action_type == AbstractAction:
            cb_dict = {
                'goal_response_callback': self.default_goal_response_callback,
                'callback_group': self.callback_group,
                'feedback_callback': self.default_feedback_callback,
                'result_callback': self.default_result_callback}
        else:
            self.simulation_node.get_logger().error(
                'action type does not match callbacks specification')
            return
        return cb_dict

    # ---------------------- Default Callbacks for ActionClient ---------------
    def default_send_goal(self, goal_msg):
        # blocking call, some prudence maybe necessary for real xp (ex: check the connection before call)
        self.client_test.wait_for_server()
        self._send_goal_future = self.client_test.send_goal_async(
            goal_msg, feedback_callback=self.default_feedback_callback)
        self._send_goal_future.add_done_callback(
            self.default_goal_response_callback)

    def default_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.partial_sequence))

    def default_goal_response_callback(self, future):
        print('future cb')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.default_result_callback)

        self.goals_results.append(result_future)

    def default_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        # self.plan = self.plan[1:]
        self.execute_bundles_callback()

    # ---------------------- GoTo Callbacks for ActionClient -----------------
    def goto_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.partial_sequence))

    def goto_goal_response_callback(self, future):
        print('future cb')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('GoTo Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goto_result_callback)

        self.goals_handles.append(goal_handle)
        self.goals_results.append(result_future)

    def goto_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.robot_plan.finish_action(result.goal_id)
        # retirer le goal terminé de goals_handles
        self.execute_bundles_callback()

    # ------------------ AbstractAction Callbacks for ActionClient ------------
    def abstract_action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.partial_sequence))

    def abstract_action_goal_response_callback(self, future):
        print('future cb')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('AbstractAction Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.abstract_action_result_callback)

        self.goals_results.append(result_future)

    def abstract_action_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.robot_plan.finish_action(result.goal_id)
        self.execute_bundles_callback()
    # ----------------- end ActionClient Callbacks specification --------------

    # -------------------INITIALIZE LISTENERS AND PUBLISHERS ------------------
    def initialize_listeners(self, init_gt=True, init_auction=True):
        """
        Initialize all listeners of the robot
        """
        # ------------------ Ground truth topics
        if init_gt:
            self.initialize_ground_truth_listeners()

        # ------------------ Auctions topics
        if init_auction:
            self.initialize_auctions_listeners()

    def initialize_publishers(self,  init_auction=True):
        """
        Initialize all publishers of the robot
        """
        # ------------------ Auctions topics
        if init_auction:
            self.initialize_auctions_publishers()

    def initialize_actions_clients(self):
        """"Initialize all actions clients of the robot
        """
        # ------------- Robot actions clients to interact with functional layer
        clients_ns = 'fl_actions'
        # Initialize dicts
        self.node_actions_clients[clients_ns] = {}

        # Create actions clients
        if 'actions' in self.robot_spec_data:
            robot_actions = self.robot_spec_data['actions']
            for action_name in robot_actions:
                action_type = get_actions.get_action_type(
                    action_name, self.fl_actions_package_name)
                if not action_type:
                    continue
                    # TODO replace by exception
                # print('initialize server', self.robot_id, ':', action_name)
                action_client_name = utils.str_builder(
                    [clients_ns, action_name], '/')
                cb_dict = self.get_action_client_cb(action_type)
                client = ActionClient(self, action_type, action_client_name,
                                      callback_group=cb_dict['callback_group'],
                                      )
                client_hdl = ClientHandler(action_name, action_type,
                                           action_client_name, client)
                self.node_actions_clients[clients_ns][action_name] = client_hdl

    def initialize_ground_truth_listeners(self):
        """
        Initialize the node's listeners. interface_node listens to the
        ground_truth topics from sim_robots for the robots' pose and energy
        status.
        """
        # Ground truth topics
        topic_ns = 'ground_truth'

        # Pose
        topic_info = 'pose'
        msg_type = Pose
        listener_cb = self.update_pose_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=10,
                                        ns_list=[[topic_ns]])
        # Energy
        topic_info = 'energy'
        msg_type = Float64
        listener_cb = self.update_ener_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=10,
                                        ns_list=[[topic_ns]])

    def update_pose_cb(self, msg, topic_name=None):
        """
        Callback called when receiving a pose message. Upon reception, updates
        the position of the robot.
        :param msg: ROS2 msg of type Pose, the message stating updated pose
          data.
        :param topic_name:
        """
        self.get_logger().info(f"Received position {msg.position.x, msg.position.y}.")
        self.last_pos_update
        self.position = (msg.position.x, msg.position.y)

    def update_ener_cb(self, msg, topic_name=None):
        """
        Callback called when receiving an energy message. Upon reception,
        updates the energy of the robot.
        :param msg: ROS2 msg of type Float64, the message stating updated
          energy data.
        :param topic_name:
        """
        self.energy = msg.data

    def initialize_auctions_listeners(self):
        """
        Initialize auctions listeners
        """
        topic_ns = 'auctions'
        # General announcement topic
        topic_info = 'announcement'
        msg_type = AnnouncementStamped
        # listener_cb = self.auctions_msg_cb
        listener_cb = self.auction_scheme.announcement_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb, queue_size=10,
                                        ns_list=[[topic_ns]])

        # General bid topic
        topic_info = 'bid'
        msg_type = BidStamped
        # listener_cb = self.auctions_msg_cb
        listener_cb = self.auction_scheme.bid_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb, queue_size=10,
                                        ns_list=[[topic_ns]])

        # General result topic
        topic_info = 'results'
        msg_type = ResultsStamped
        # listener_cb = self.auctions_msg_cb
        listener_cb = self.auction_scheme.results_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb, queue_size=10,
                                        ns_list=[[topic_ns]])

        # General award acknowledgement topic
        topic_info = 'award_acknowledgement'
        msg_type = AwardAcknowledgementStamped
        # listener_cb = self.auctions_msg_cb
        listener_cb = self.auction_scheme.award_acknowledgement_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb, queue_size=10,
                                        ns_list=[[topic_ns]])

    def initialize_auctions_publishers(self):
        topic_ns = 'auctions'
        # General announcement topic
        topic_info = 'announcement'
        msg_type = AnnouncementStamped
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers,
                                         queue_size=10,
                                         ns_list=[[topic_ns]])

        # General bid topic
        topic_info = 'bid'
        msg_type = BidStamped
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers,
                                         queue_size=10,
                                         ns_list=[[topic_ns]])

        # General results topic
        topic_info = 'results'
        msg_type = ResultsStamped
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers,
                                         queue_size=10,
                                         ns_list=[[topic_ns]])

        # General award acknowledgement topic
        topic_info = 'award_acknowledgement'
        msg_type = AwardAcknowledgementStamped
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers,
                                         queue_size=10,
                                         ns_list=[[topic_ns]])

    def send_bids(self, bids_msgs_stamped):
        """
        Sends bids to teammates
        :param bids_msgs_stamped:
        """
        for bid_msg_stamped in bids_msgs_stamped:
            self.send_msg_to_teammates(bid_msg_stamped)

    def execute_bundles_callback(self):
        """
        Callback called when obtained_bundles need to be planned.
        The timer setting allows to wait potential others bundles before
        planning it. This callback could also be called at the end of an
        auction session and not directly after a result
        (which could be a auction tour result)

        NEEDS to be surcharged for specific planning and execution

        :return:
        """
        # Reset timer
        # self.timer_plan.cancel()

        self.robot_plan.plan_items()

        try:
            # execute first action
            self.robot_plan.execute_next_actions()
        except:
            self.get_logger().error(traceback.format_exc())

    def send_goal(self, goal_msg):
        """
        This method send action goal
        NEEDS to be surcharged with your use case
        :param goal_msg: goal message to send
        """
        # print('SEND GOAL')
        client_ns = 'fl_actions'

        if isinstance(goal_msg, GoTo.Goal):
            action_name = 'GoTo'
            feedback_callback = self.goto_feedback_callback
            goal_response_callback = self.goto_goal_response_callback
        elif isinstance(goal_msg, AbstractAction.Goal):
            action_name = 'AbstractAction'
            feedback_callback = self.abstract_action_feedback_callback
            goal_response_callback = self.abstract_action_goal_response_callback
        else:
            self.get_logger().warn(
                'Impossible to find a matching ActionClient')
            return
        # blocking call, some prudence maybe necessary for real xp
        # (ex: check the connection before call)
        client = self.node_actions_clients[client_ns][action_name].client

        client.wait_for_server()  # TODO manage this blocking call
        self._send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback)
        self._send_goal_future.add_done_callback(goal_response_callback)

    def publish_msg(self, msg, topic_keys):
        """
        This function publish a message on a topic.
        :param msg: message to publish
        :param topic_keys: keys to retrieve publisher in node publishers dict
        :return:
        """
        msg.header.message_id = self.robot_id + "_" + str(self.msg_id_cpt)
        self.msg_id_cpt += 1
        topic = utils.get_dict_value_list_keys(
            self.node_publishers, topic_keys)
        if not topic:
            self.get_logger().error("Impossible to publish message")
            return
        else:
            topic.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Robot()

    executor = MultiThreadedExecutor(num_threads=20)
    rclpy.spin(node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
