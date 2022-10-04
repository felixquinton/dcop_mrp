"""Describes a robot node. The Robot node takes a parameter robot_id that MUST
correspond to a unique ComSim node's robot_id parameter to form a ComSim-Robot
pair. The Robot node randomly generate announcement messages that it sends to
its ComSim. It is also able to receive annoucement messages and bid messages
from the ComSim. While bid messages are ignored in the present version,
annoucement messages trigger a bid message that is sent to the ComSim.
QOS: Quality Of Service for messsage loss
"""

import posixpath
from turtle import position
import rclpy
import json
import os
import traceback
import numpy as np
import networkx as nx
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from caf_robot.robot_member_function import Robot as BaseRobot
from patrol_essential.utils import utils
from caf_essential.base_node import BundleHandlerBidder
from patrol_essential.auction import navigation_graph as nv
from patrol_messages.msg import (LastVisitsStamped, LastVisits, Header,
                                 TeamDestStamped, TeamDest, EditEdge, SendTour,
                                 EditEdgeStamped, EditNode, EditNodeStamped)
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from caf_messages.msg import BundleDescription, ItemDescription, Bid
from patrol_essential.auction import single_item_auction as si
from patrol_essential.auction import sequential_single_item_auction as ssi
from patrol_essential.planning import simple_sequential_planning
from patrol_essential.utils import utils as putils

patrol_msg_types = [LastVisitsStamped, TeamDestStamped]
dynamic_events_msg_types = [EditEdgeStamped, EditNodeStamped]


class Robot(BaseRobot):
    """
    Class that manage all of the robot process
    TODO split planning, supervision, auction in others structures ?
    """

    def __init__(self):
        """Initialize the Robot. The parameter robot_id is saved to form a pair
        with the robot's ComSim. Subscriptions and publishers are set up
        according to this parameter to ensure a one to one communication link
        between the robot and its ComSim.
        Parameters
        ----------
        Returns
        ----------
        """
        self.simulation_time = 0
        self.last_pos_change = self.simulation_time
        self.current_auctioned_task = None
        self.must_replan = False
        self.to_plan_later = []
        self.shadow_target = None

        super().__init__()
        communication_range, scenario_path = self._get_parameters()
        if self.auction_scheme:
            self.auction_scheme._init_first_path(self.first_path)
        self.auctioneer_auction_to_start = []
        self.auction_cpt = 0
        self.auctioneer_in_a_row_cpt = 0
        self.is_initial_auctioneer = (
            True if self.robot_id == "robot_1" else False)
        self.current_auction_auctioneer = None
        self.team_members_data = utils.get_team_members_data(self)
        self.robots_spec_data = utils.get_robots_spec_data(self)
        self.robot_spec_data = self.robots_spec_data[self.robot_id]
        self.mission_spec_data = utils.get_mission_spec_data(self)
        self.simu_refresh = self.mission_spec_data["simu_period"]
        self.simu_speed = self.mission_spec_data["time_factor"]
        if self.auction_scheme:
            self.auction_scheme.__init_simu_speed__()
        self.robot_spec_data = self.robots_spec_data[self.robot_id]
        self.energy_cons = self.robot_spec_data["robot_energy_cons"]
        self.max_speed = self.robot_spec_data["robot_max_speed"]
        self.robot_type = self.robot_spec_data["robot_type"]
        self.robot_sensors = self.robot_spec_data["robot_sensors"]
        self.get_logger().info(f"Sensors: {self.robot_sensors}")

        if scenario_path:
            self._init_data_from_scenario_file(
                scenario_path, communication_range)
        else:
            self._init_data_from_mission_specs()

        self.navigation_graph = self.nav_graph
        self.robot_plan = simple_sequential_planning.SimpleSequentialPlan(self)

        self.last_visits = self._initialize_last_visits()
        self.waypoint_targets = []
        self.teammate_poses = {r: self.robots_spec_data[r]["robot_spawn_pos"]
                               for r in self.robots_spec_data}

        # stay_and_survey: If not None, the robot survey its position,
        # continuously resetting the idleness to 0.
        self.com_graph = None
        self.initialize_ground_truth_listeners()
        self.stay_and_survey = None
        self.stay_and_survey_timer = self.create_timer(
            1.0, self.survey_surroundings,
            callback_group=self.callback_group)
        self.team_dest = self._initialize_team_dest()

        self.robot_max_speed = self.robot_spec_data["robot_max_speed"]

        self.get_logger().info("Init ok.")

    def _get_parameters(self):
        self.declare_parameter("communication_range", 1)
        communication_range = self.get_parameter("communication_range").value

        self.declare_parameter("scenario_path", "")
        scenario_path = self.get_parameter("scenario_path").value

        self.declare_parameter("auction_com_cost", "")
        self.auction_com_cost = self.get_parameter("auction_com_cost").value

        self.declare_parameter("scenario_id")
        self.folder_str = self.get_parameter("scenario_id").value

        self.declare_parameter("folder")
        self.first_path = self.get_parameter("folder").value + "/"
        return communication_range, scenario_path

    def _init_data_from_mission_specs(self):
        self.nav_graph = nv.navGraph(
            self.mission_spec_data["graph"])
        self.robots_com_range = {
            r_id: self.robots_spec_data[r_id]["com_range"]
            for r_id in self.robots_spec_data}
        self.position = self.robots_spec_data[
            self.robot_id]['robot_spawn_pos']
        self.edge_events = self.mission_spec_data["edge_events"]
        self.node_events = self.mission_spec_data["node_events"]

    def _init_data_from_scenario_file(
            self, scenario_path, communication_range):
        data_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            scenario_path)
        with open(data_path) as datafile:
            data = json.loads(json.load(datafile))

        self.nav_graph = nv.navGraph(
            utils.import_generated_nav_graph(data))

        self.robots_com_range = {
            r_id: communication_range for r_id in self.robots_spec_data}

        self.position = self.nav_graph.nodes[0]["pos"]
        self.edge_events = []  # At the moment there is no edge event.
        self.node_events = data["node_events"]
        self.proba_obs_data = data["edge_to_edge_com"]

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
                item = ItemDescription()
                item.item_id = 'item_'+str(waypoint)
                item_type = 'GoTo'
                if item_type == 'GoTo':
                    item.item_name = 'goto_start_pos'
                    item.item_type = 'GoTo'
                    item.item_data = ('{"target_position": {"x": '
                                      + str(float(positions[waypoint][0]))
                                      + ', "y": '
                                      + str(float(positions[waypoint][1]))
                                      + ', "z": 0.0}}')
                else:
                    item.item_name = 'abs'
                    item.item_type = 'AbstractAction'
                    item.item_data = '{"target_duration": 4.5}'
                # Define second item to sell
                self.auctioneer_auction_to_start.append(item)

        # Timer to publish auctioneer token requests and surrender.
        self.auctioneer_in_a_row_cpt = 0
        if not self.method == "DCOP":
            self.request_pass_token = self.create_timer(
                self.simu_refresh/2, self._token_speaker,
                callback_group=self.callback_group)

    def _pos_sharing_cb(self):
        team_dest_msg = TeamDest()
        team_dest_msg.robot_id = self.robot_id
        team_dest_msg.team_dest = json.dumps(
            self.team_dest)
        team_dest_msg.robot_position = Point()
        team_dest_msg.robot_position.x = self.position[0]
        team_dest_msg.robot_position.y = self.position[1]
        team_dest_stp = TeamDestStamped()
        team_dest_stp.header = Header()
        team_dest_stp.header.sender_id = self.robot_id
        team_dest_stp.header.required_acknowledgement = False
        team_dest_stp.body_msg = team_dest_msg
        self.send_msg_to_teammates(team_dest_stp)

    def _set_auction_scheme(self):
        if self.method in ["SSI", "SI"]:
            # SI and SSI auctions are equivalent given the valuation schemes
            # used here, i.e., SIAs are actually SSIAs.
            # self.auction_scheme = ssi.SSIAuction(self)
            self.method = "SI"
            self.auction_scheme = si.PatrolSIAuction(self)
        elif self.method == "DCOP":
            self.auction_scheme = None
        else:
            self.auction_scheme = si.PatrolSIAuction(self)

    def survey_surroundings(self):
        self.get_logger().info(f"Stay and survey: {self.stay_and_survey}")
        # If the robot stays at same waypoint for a long time, survey it.
        if self.simulation_time - self.last_pos_change > 60:
            self.stay_and_survey = self.nav_graph.convert_pos_into_node(
                self.position, 0.1)

        self.get_logger().warn(f"Waypoint targets was {self.waypoint_targets}")
        self.get_logger().warn(f"Tasks to auction were {self.auctioneer_auction_to_start}")
        if self.method == "SI":
            if self.waypoint_targets:
                if not self.robot_plan.actions_order:
                    self.plan_waypoints_by_id(self.waypoint_targets[0])


        if self.stay_and_survey is not None:
            waypoint_id = self.nav_graph.convert_pos_into_node(
                self.position, 0.1)
            if waypoint_id is not None:
                goto_request_termination_msg = String()
                goto_request_termination_msg.data = self.robot_id
                self.goto_request_termination_pub.publish(
                    goto_request_termination_msg)
                self.get_logger().info("Sent goto request termination")
                self._check_node_events(waypoint_id)
                ws = self.nav_graph.nodes[waypoint_id]["sensors"]
                if any(s in ws for s in self.robot_sensors):
                    self.last_visits[waypoint_id] = self.simulation_time

                    if (self.waypoint_targets
                            and waypoint_id == self.waypoint_targets[0]):
                        self.waypoint_targets.remove(waypoint_id)

                    last_visits_stp = self._fill_last_visits_msg()

                    self.send_msg_to_teammates(last_visits_stp,
                                               surveyed=waypoint_id)
                else:
                    self.get_logger().info(
                        f"Attempted to survey waypoint {waypoint_id} but "
                        f"sensor requierement was {ws} and robot had sensors"
                        f" {self.robot_sensors}.")

                    if (self.waypoint_targets
                            and waypoint_id == self.waypoint_targets[0]):
                        self.waypoint_targets.remove(waypoint_id)
            else:
                self.get_logger().error(
                    f"Attempted to survey surroundings while robot is not at"
                    f" any waypoint's position. Robot's position: "
                    f"{self.position}")

    def goto_result_callback(self, future):
        try:
            result = future.result().result

            self.get_logger().info('Result: {0}'.format(result))

            self.robot_plan.finish_action(result.goal_id,
                                          execute_next_action=False)

            waypoint = self.nav_graph.convert_pos_into_node((
                result.reached_position.x, result.reached_position.y))
            waypoint_pos = (float(result.reached_position.x),
                            float(result.reached_position.y))

            if self.waypoint_targets:
                current_target = self.waypoint_targets[0]
            else:
                current_target = None

            self.get_logger().info(f"Robot pos before: {self.position}.")
            self.position = waypoint_pos
            self.get_logger().info(f"Robot pos after: {self.position}.")
            # If the waypoint is a target different from the shadow target,
            # but in the current targets, discard the shadow target
            """
            if (self.shadow_target is not None
                    and waypoint in self.waypoint_targets
                    and waypoint != self.shadow_target
                    and self.waypoint_targets[0] == self.shadow_target):
                self.waypoint_targets.remove(self.shadow_target)
                self.shadow_target = None
            """
            
            # Define item to sell
            item = ItemDescription()
            item.item_id = 'item_'+str(waypoint)
            item_type = 'GoTo'
            if item_type == 'GoTo':
                item.item_name = 'goto_start_pos'
                item.item_type = 'GoTo'
                item.item_data = ('{"target_position": {"x": '
                                  + str(float(waypoint_pos[0]))
                                  + ', "y": '
                                  + str(float(waypoint_pos[1]))
                                  + ', "z": 0.0}}')

            if self.method == "DCOP":
                self.get_logger().info(f"DCOP ALLOC {self.current_dcop_alloc}")
                self.get_logger().info(f"WAYPOINT {waypoint}")
                self.get_logger().info(
                    f"WAYPOINTS TARGETS {self.waypoint_targets}")
                """
                if len(self.waypoint_targets) > 1 and waypoint == self.waypoint_targets[1] and self.waypoint_targets[0] not in self.current_dcop_alloc:
                    self.get_logger().info(f"Removing waypoint {self.waypoint_targets[0]} from waypoint targets {self.waypoint_targets} because already at {waypoint}.")
                    self.waypoint_targets = self.waypoint_targets[1:]
                    self.get_logger().info(f"Resulting in {self.waypoint_targets}")
                """


            if self.waypoint_targets and waypoint == self.waypoint_targets[0]:
                if self.method == "SSI":
                    self.auction_scheme.auction_to_repeat.append(item)
                elif self.method == "DCOP":
                    self.waypoint_targets = self.waypoint_targets[1:]
                    self.waypoint_targets.append(waypoint)
                    self.plan_waypoints_by_id(
                        [self.waypoint_targets[0]])
                    for item_hdl in self.to_plan_later:
                        # self.plan_waypoints_by_id(
                        #     int(item_hdl.item_id.split('_')[-1]))
                        self.waypoint_targets.append(
                            int(item_hdl.item_id.split('_')[-1]))

            # surveyed variable used to convey which waypoint has been surveyed
            surveyed = None

            # Reset idleness if in waypoint targets
            if self.waypoint_targets and waypoint == current_target:
                self._check_node_events(waypoint)
                waypoint_sensors = self.nav_graph.nodes[waypoint]["sensors"]
                if any(s in waypoint_sensors for s in self.robot_sensors):
                    self.last_visits[waypoint] = self.simulation_time
                    surveyed = waypoint

                if (round(self.team_dest[waypoint]["date"], 2)
                        != round(self.simulation_time, 2)):
                    self.get_logger().warn(
                        f"""Waypoint {waypoint} was reached at time
                        {self.simulation_time} but scheduled at
                        {self.team_dest[waypoint]}""")
                if self.method != "DCOP":
                    self.waypoint_targets.remove(waypoint)

                if ('item_'+str(waypoint) not in [
                        obj.item_id for obj in
                        self.auctioneer_auction_to_start]
                        and self.auction_scheme):
                    self.auctioneer_auction_to_start.append(item)

                    dir = f"{self.first_path}{self.folder_str}"
                    file_name = "/auction_to_start_log.csv"
                    data_list = [self.simulation_time,
                                 self.robot_id,
                                 "goto_res",
                                 self.auctioneer_auction_to_start]
                    try:
                        putils.add_row_to_csv(dir, file_name, data_list)
                    except Exception:
                        raise

            last_visits_stp = self._fill_last_visits_msg()

            self.send_msg_to_teammates(last_visits_stp, surveyed=surveyed)

            self._handle_edge_events(waypoint)

            if self.must_replan:
                # TODO: actually replan...
                # self.robot_plan.replan()
                # self.evaluate_team_dest()
                self.must_replan = False

            self.get_logger().info("Already moving on !")

            self.execute_bundles_callback()

        except Exception:
            self.get_logger().error(traceback.format_exc())

    def _handle_edge_events(self, waypoint):
        if self.robot_plan.actions_order:
            next_action_id = self.robot_plan.actions_order[0]
            action_hdl = self.robot_plan.actions[next_action_id]
            next_waypoint = self.nav_graph.convert_pos_into_node(
                (action_hdl.goal_request.target_position.x,
                 action_hdl.goal_request.target_position.y))
            for event_dic in self.edge_events:
                if (self.simulation_time > event_dic["event_date"]
                        and waypoint in event_dic["event_edge"]
                        and next_waypoint in event_dic["event_edge"]
                        and (waypoint, next_waypoint) in self.nav_graph.edges):
                    self.get_logger().info(
                        f"""Robot {self.robot_id} discovered it cannot use
                        edge {(waypoint, next_waypoint)}
                        {type(action_hdl.goal_request.target_position)}""")
                    self.nav_graph.remove_edge(waypoint, next_waypoint)
                    self.publish_edit_edge_msg(event_dic)

    def publish_edit_edge_msg(self, event_dic):
        edge_msg = EditEdge()
        edge_msg.simulation_time = self.simulation_time
        edge_msg.source, edge_msg.target = event_dic["event_edge"]
        edge_msg.edit_type = event_dic["event_type"]
        edge_msg.robot_types = event_dic["robot_types"]

        edge_msg_stp = EditEdgeStamped()
        edge_msg_stp.header = Header()
        edge_msg_stp.header.sender_id = self.robot_id
        edge_msg_stp.body_msg = edge_msg

        # publish msg
        self.send_msg_to_teammates(edge_msg_stp)

    def execute_bundles_callback(self):
        """
        Callback called when obtained_ need to be planned.
        The timer setting allows to wait potential others  before
        planning it. This callback could also be called at the end of an
        auction session and not directly after a result
        (which could be a auction tour result)

        NEEDS to be surcharged for specific planning and execution

        :return:
        """
        self.robot_plan.plan_items()

        try:
            # execute first action
            self.robot_plan.execute_next_actions()
            self.get_logger().info("Finished execute next actions")
            if not self.robot_plan.actions.values() and self.stay_and_survey is None:
                goto_request_termination_msg = String()
                goto_request_termination_msg.data = self.robot_id
                self.goto_request_termination_pub.publish(
                    goto_request_termination_msg)
                self.get_logger().info("Sent goto request termination")
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def _start_auction_if_avalaible(self):
        try:
            if len(self.auctioneer_auction_to_start) == 0:
                if self.method == "SSI":
                    for task in self.auction_scheme.auction_to_repeat:
                        self.auctioneer_auction_to_start.append(task)
                    self.auction_scheme.auction_to_repeat = []
            if self.current_auction_bidder is None:
                if self.current_auction_auctioneer is None:
                    if self.auction_scheme.auctioneer_token:
                        if len(self.auctioneer_auction_to_start) > 0:
                            self.auction_scheme.send_announcement()

                            self.current_auctioned_task = \
                                self.auctioneer_auction_to_start[0]
                            self.auctioneer_auction_to_start.pop(0)
                            self.auctioneer_in_a_row_cpt += 1
                            self.auction_cpt += 1
        except Exception:
            self.get_logger().error("From start_auction"
                                    + traceback.format_exc())

    def _token_speaker(self):
        # Important parameter that handles the number of consecutive auctions
        # a robot can start.
        max_auctioneer_cpt = int(
            len(self.nav_graph.nodes)/len(self.robots_spec_data))
        if self.is_initial_auctioneer:
            max_auctioneer_cpt = len(self.nav_graph.nodes)
        if self.auctioneer_in_a_row_cpt == len(self.nav_graph.nodes)-1:
            self.is_initial_auctioneer = False
        if (len(self.auctioneer_auction_to_start) > 0
                and self.auctioneer_in_a_row_cpt < max_auctioneer_cpt):
            request_msg = String()
            request_msg.data = self.robot_id
            self.request_pub.publish(request_msg)
        if (len(self.auctioneer_auction_to_start) == 0
                or self.auctioneer_in_a_row_cpt >= max_auctioneer_cpt):
            self.auctioneer_in_a_row_cpt = 0
            surr_msg = String()
            surr_msg.data = self.robot_id
            self.surr_pub.publish(surr_msg)
            self.auction_scheme.auctioneer_token = False

    def _fill_last_visits_msg(self):
        last_visits_msg = LastVisits()
        last_visits_msg.robot_id = self.robot_id
        last_visits_msg.last_visits = json.dumps(self.last_visits)
        last_visits_stp = LastVisitsStamped()
        last_visits_stp.header = Header()
        last_visits_stp.header.sender_id = self.robot_id
        last_visits_stp.header.required_acknowledgement = False
        last_visits_stp.body_msg = last_visits_msg
        return last_visits_stp

    def initialize_listeners(self, init_gt=True, init_auction=True):
        # The use of init_auction and self.method to initiate auction listeners
        # is not satifying.
        self.method = self.get_parameter("method").value

        self.initialize_ground_truth_listeners()
        if self.method in ["SI"]:
            self.initialize_auctions_listeners()
        self.initialize_patrol_listeners()
        self.initialize_event_listeners()
        self.initialize_dynamic_events_listeners()
        self.initialize_dcop_plans_listeners()

    def initialize_publishers(self):
        super().initialize_publishers(init_auction=(self.method in ["SI"]))
        self.initialize_patrol_publishers()
        self.initialize_token_publishers()
        self.initialize_auction_termination_publishers()
        self.initialize_goto_request_termination_publishers()
        self.initialize_dynamic_events_publishers()

    def initialize_auction_termination_publishers(self):
        topic_ns = 'auction_termination'
        topic_info = 'reset_auction_timer'
        msg_type = String
        self.auction_termination_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def initialize_goto_request_termination_publishers(self):
        topic_ns = 'goto_goal_request'
        topic_info = 'reset_timer'
        msg_type = String
        self.goto_request_termination_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def publish_auction_termination(self):
        """Send a termination message to the sim to indicate that a robot
        closed an auction.
        """
        auction_termination_msg = String()
        auction_termination_msg.data = "Auction closed."
        self.auction_termination_pub.publish(auction_termination_msg)

    def initialize_event_listeners(self):
        # Ground truth topics
        topic_ns = 'event'

        # Send shared data messages to teammates
        topic_info = 'share_data'
        msg_type = String
        listener_cb = self.receive_simu_event
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=10,
                                        ns_list=[[topic_ns]])

        # Receive auctioneer token from token dispenser
        topic_info = 'auctioneer_token'
        msg_type = String
        listener_cb = self.receive_distribution
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=10,
                                        ns_list=[[topic_ns]])

    def receive_edit_edge(self, msg, topic_name=None):
        body_msg = msg.body_msg
        if body_msg.edit_type == "remove":
            try:
                self.nav_graph.remove_edge(body_msg.source, body_msg.target)
            except Exception as e:
                self.get_logger().error(f"Removal failed with error {e}")

    def receive_edit_node(self, msg, topic_name=None):
        # self.get_logger().info(f"Received an edit node msg {msg}")
        body_msg = msg.body_msg
        if body_msg.edit_type == "edit_sensors":
            try:
                self.nav_graph.edit_nodes_sensors(
                    {body_msg.node_id: body_msg.sensor_types})
            except Exception as e:
                self.get_logger().error(
                    f"""Sensor edition failed on node {body_msg.node_id}
                    with error {e}""")

        events_to_remove = []
        for event in self.node_events:
            if event["event_node"] == body_msg.node_id:
                events_to_remove.append(event)
        for event in events_to_remove:
            if event in self.node_events:
                self.node_events.remove(event)
        self.get_logger().info(f"Updated node events {self.node_events}")

    def receive_simu_event(self, msg, topic_name=None):
        if msg.data == "share_data":
            self._pos_sharing_cb()

    def receive_distribution(self, msg, topic_name=None):
        if msg.data == self.robot_id:
            self.auction_scheme.auctioneer_token = True
            self._start_auction_if_avalaible()

    def initialize_token_publishers(self):
        topic_ns = 'token'
        topic_info = 'request'
        msg_type = String
        self.request_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

        topic_info = 'pass'
        msg_type = String
        self.surr_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def initialize_event_publishers(self):
        topic_ns = 'event'
        topic_info = 'auction_finished'
        msg_type = String
        self.auction_finish_pub = self.create_publisher(
            msg_type, '/'+topic_ns+'/'+topic_info, 10)

    def _initialize_last_visits(self):
        start_time = self.simulation_time
        res = {}
        for node in self.nav_graph.nodes:
            res[node] = start_time
        return res

    def _initialize_team_dest(self):
        res = {}
        for node in self.nav_graph.nodes:
            res[node] = {"robot_id": None, "date": 0}
        return res

    def initialize_patrol_publishers(self):
        topic_ns = 'patrol'
        # Last visits
        topic_info = 'last_visits'
        msg_type = LastVisitsStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[topic_ns]])
        # Team dest
        topic_info = 'team_dest'
        msg_type = TeamDestStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers, queue_size=1000,
            ns_list=[[topic_ns]])

    def initialize_patrol_listeners(self):
        topic_ns = 'patrol'
        # Last visits
        topic_info = 'last_visits'
        msg_type = LastVisitsStamped
        listener_cb = self.update_last_visits
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])
        # Team dest
        topic_info = 'team_dest'
        msg_type = TeamDestStamped
        listener_cb = self.update_team_dest
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=1000, ns_list=[[topic_ns]])

    def initialize_dcop_plans_listeners(self):
        """
        Initialize dcop related listeners
        """
        topic_ns = 'dcop'
        # Receive dcop plans as JSON strings
        topic_info = 'dcop_plan'
        msg_type = SendTour
        listener_cb = self.receive_dcop_plan
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

    def initialize_dynamic_events_listeners(self):
        """
        Initialize edit edge listeners
        """
        topic_ns = 'event'
        # General edit edge topic
        topic_info = 'edit_edge'
        msg_type = EditEdgeStamped
        listener_cb = self.receive_edit_edge
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

        # General edit node topic
        topic_info = 'edit_node'
        msg_type = EditNodeStamped
        # listener_cb = self.auctions_msg_cb
        listener_cb = self.receive_edit_node
        self.create_and_store_listeners(
            topic_info, msg_type, self.node_listeners,
            listener_cb=listener_cb, queue_size=10, ns_list=[[topic_ns]])

    def initialize_dynamic_events_publishers(self):
        topic_ns = 'event'
        # General edit edge topic
        topic_info = 'edit_edge'
        msg_type = EditEdgeStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[topic_ns]])

        # General edit node topic
        topic_info = 'edit_node'
        msg_type = EditNodeStamped
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=10, ns_list=[[topic_ns]])

    def update_last_visits(self, msg, topic_name=None):
        """
        Callback called when receiving a last visits message. Upon reception,
        updates corresponding dict of the robot.
        :param msg: ROS2 msg of type LastVisitsStamped, the message stating
        updated pose data.
        :param topic_name:
        """
        tmp = json.loads(msg.body_msg.last_visits)
        for waypoint, visit_time in tmp.items():
            waypoint = int(waypoint)
            if visit_time > self.last_visits[waypoint]:
                self.last_visits[waypoint] = visit_time

    def update_pose_cb(self, msg, topic_name=None):
        """
        Callback called when receiving a pose message. Upon reception, updates
        the position of the robot.
        :param msg: ROS2 msg of type Pose, the message stating updated pose
          data.
        :param topic_name:
        """
        self.get_logger().info(f"Received position {msg.position.x, msg.position.y}.")
        previous_pos = self.position
        current_pos = (msg.position.x, msg.position.y)
        if previous_pos != current_pos[0]:
            self.last_pos_change = self.simulation_time
        else:
            self.get_logger().info(f"Increasing self.last_pos_change: {self.simulation_time, self.last_pos_change}.")
        self.position = current_pos

    def receive_dcop_plan(self, msg, topic_name=None, verbose=True):
        """
        Callback called when receiving a DCOP plan. Upon reception,
        updates the plan of the robot.
        :param msg: ROS2 msg of type TeamDestStamped, the message stating
        updated pose data.
        :param topic_name:
        """
        if verbose or True:
            self.get_logger().info(f"I received this DCOP message {msg}. \n")
        digit_id = int(self.robot_id.split('_')[-1])
        all_tours = json.loads(msg.tours)
        # If robot is not is the dcop solution, ignore message.
        if str(digit_id) not in all_tours:
            return

        tasks = all_tours[str(digit_id)]
        self.current_dcop_alloc = [t for t in tasks]
        if verbose:
            self.get_logger().info(f"Tasks {tasks}")
            self.get_logger().info(f"DCOP ALLOC {self.current_dcop_alloc}")
            self.get_logger().info(f"Obtained before {self.obtained_bundles}")
            self.get_logger().info(
                f"Plan before {self.waypoint_targets}")

        self.get_logger().info("LA")

        # Rarely tasks vanished and I don't know how.
        # I add them arbitrarily here...
        """
        if digit_id in all_tours.keys():
            for waypoint in self.waypoint_targets:
                vanished_waypoint = True
                for tour in all_tours.values():
                    for w in tour:
                        if waypoint == w:
                            vanished_waypoint = False
                if vanished_waypoint:
                    tasks[digit_id].append(waypoint)
                    self.get_logger().warn(
                        f"Had to append waypoint {waypoint} as it vanished.")
        """

        # self.plan_waypoints_by_id(tasks, reset=True)
        if self.waypoint_targets:
            self.waypoint_targets = [self.waypoint_targets[0]] + self.current_dcop_alloc
        else:
            self.waypoint_targets = self.current_dcop_alloc
        if not self.robot_plan.actions_order:
            self.plan_waypoints_by_id(self.waypoint_targets[0])

        self.get_logger().info("APRES")

        if verbose:
            self.get_logger().info(f"Obtained after {self.obtained_bundles}")
            self.get_logger().info(f"Obtained after {self.robot_plan.actions_order}")
            self.get_logger().info(
                f"Plan after {self.waypoint_targets}")

    def plan_waypoints_by_id(self, tasks, reset=False, eps=0.10, position_reached=None):

        self.get_logger().info(f"ICI, with reset = {reset}, tasks = {tasks}, targets = {self.waypoint_targets}.")

        positions = nx.get_node_attributes(self.nav_graph, "pos")

        # In some case a single tasks is presented as an int.
        if not isinstance(tasks, list):
            if isinstance(tasks, int):
                tasks = [tasks]
            else:
                tasks = list(tasks)

        # If first target is robot's position, put it at the end of the tour
        if tasks:
            first_target = tasks[0]
            first_position = positions[first_target]
            self.get_logger().info(f"CHERCHE ICI: {first_target, first_position}")
            if ((float(first_position[0])-self.position[0])**2
                    +(float(first_position[1])-self.position[1])**2)**.5 < eps:
                if len(self.waypoint_targets) > 1:
                    # Handling rare instances in which waypoitn targets includes only
                    # occurences of the same waypoint.
                    if self.waypoint_targets[1] == first_target:
                        self.waypoint_targets = self.waypoint_targets[1:]
                    self.waypoint_targets = self.waypoint_targets[1:] + [self.waypoint_targets[0]]
                    self.plan_waypoints_by_id([self.waypoint_targets[0]])
                    return

        # Adding tasks to robot's plan
        for w_id in list(tasks):
            # If target not in DCOP alloc, drop it.
            """
            if w_id not in self.current_dcop_alloc:
                continue
            """
            bid_msg = Bid()
            bid_msg.bid = [0.0]
            item_data = ('{"target_position": {"x": '
                        + str(float(positions[w_id][0]))
                        + ', "y": '
                        + str(float(positions[w_id][1]))
                        + ', "z": 0.0}}')
            item_list = [ItemDescription(item_id=f'waypoint_{w_id}',
                                        item_name=f'waypoint_{w_id}',
                                        item_type='GoTo',
                                        item_data=item_data)]
            bundle_id = np.random.randint(1e9)
            bundle_description = BundleDescription(
                bundle_id=str(bundle_id), item_bundle=item_list)
            bid_msg.bundle_description = bundle_description
            bundle_hdl = BundleHandlerBidder(bid_msg)
            self.obtained_bundles[bundle_id] = bundle_hdl
            self.get_logger().info(
                f"targets, id {self.waypoint_targets, w_id}")
            self.get_logger().info(
                f"before add items, plan was {self.robot_plan.p_items_hdl}")
            # Add target to list iif it is inside only once.
            self.robot_plan.add_items_from_bundle_hdl(bundle_hdl)
            count_w_id_occurences = self.waypoint_targets.count(w_id)
            if not count_w_id_occurences > 0:
                self.get_logger().info("I was inside")
                if count_w_id_occurences == 1:
                    if w_id == self.waypoint_targets[0]:
                        self.waypoint_targets.append(w_id)
                        self.get_logger().info("Count was 1 but w_id was in fist place.")
                    else:
                        self.get_logger().info("Count was 1 and w_id was not fist, so ignore.")
                else:
                    self.get_logger().info("Count was 0.")
                    self.waypoint_targets.append(w_id)
                self.get_logger().info(f"Et du coup waypoint_targets inclu: {self.waypoint_targets}")
            self.execute_bundles_callback()

    def update_team_dest(self, msg, topic_name=None):
        """
        Callback called when receiving a team dest message. Upon reception,
        updates corresponding dict of the robot.
        :param msg: ROS2 msg of type TeamDestStamped, the message stating
        updated pose data.
        :param topic_name:
        """
        if not msg.header.sender_id == self.robot_id:
            tmp = json.loads(msg.body_msg.team_dest)
            for waypoint in self.team_dest:
                if (tmp[str(waypoint)]["date"]
                        > self.team_dest[waypoint]["date"]):
                    self.team_dest[waypoint]["date"] = tmp[
                        str(waypoint)]["date"]
                    self.team_dest[waypoint]["robot_id"] = tmp[
                        str(waypoint)]["robot_id"]
        self.teammate_poses[msg.header.sender_id] = [
            msg.body_msg.robot_position.x, msg.body_msg.robot_position.y]

    def send_specific_msg(self, msg, **kwargs):
        """
        Send message related to auction process
        :param msg: auction message to send
        """
        if type(msg) in patrol_msg_types:
            self._send_patrol_msg(msg, **kwargs)
        elif type(msg) in dynamic_events_msg_types:
            self.get_logger().info(
                f'Sending the dynamic event msg {msg}')
            self._send_dynamic_event_msg(msg, **kwargs)

    def _send_patrol_msg(self, msg, **kwargs):
        keys = ['patrol']
        if isinstance(msg, LastVisitsStamped):
            keys.append('last_visits')

            updated_waypoint = kwargs.get("surveyed", None)
            instant_idle_dic = self.last_visits.copy()

            idle_max = 0
            idle_avg = 0
            for visit_date_w in list(instant_idle_dic.values()):
                current_idle_w = self.simulation_time - visit_date_w
                if current_idle_w > idle_max:
                    idle_max = current_idle_w
                idle_avg += current_idle_w
            idle_avg /= len(instant_idle_dic)
            instant_idle_dic["mean"] = idle_avg
            instant_idle_dic["max"] = idle_max

            dir = f"{self.first_path}{self.folder_str}"
            file_name = f"/{self.robot_spec_data['idleness_save_file']}"
            data_list = [self.robot_id,
                         str(self.simulation_time),
                         str(updated_waypoint),
                         json.dumps(instant_idle_dic)]
            try:
                putils.add_row_to_csv(dir, file_name, data_list)
            except Exception:
                raise

        elif isinstance(msg, TeamDestStamped):
            keys.append('team_dest')
        else:
            self.get_logger().error(
                'Impossible to send msg')
        self.publish_msg(msg, keys)
        # topic_name = utils.str_builder(keys, '/')
        # self.get_logger().info(
        #     'Message transferred to ' + topic_name)

    def _send_dynamic_event_msg(self, msg, **kwargs):
        keys = ['event']
        if isinstance(msg, EditEdgeStamped):
            keys.append('edit_edge')
        if isinstance(msg, EditNodeStamped):
            keys.append('edit_node')
        else:
            self.get_logger().error(
                'Impossible to send dynamic event msg')
        # self.get_logger().info(
        #     f'Just before transfer, with msg {msg} and keys {keys}')
        self.publish_msg(msg, keys)
        topic_name = utils.str_builder(keys, '/')
        self.get_logger().info(
            f"""Just after transfer, with msg {msg} topic name {topic_name},
            and keys {keys}""")

    def update_time_cb(self, msg, topic_name=None):
        self.simulation_time = msg.data

    def update_com_network_cb(self, msg, topic_name=None):
        self.com_graph = nx.readwrite.node_link_graph(json.loads(msg.data))

    def initialize_ground_truth_listeners(self):
        """
        Initialize the node's listeners. interface_node listens to the
        ground_truth topics from sim_robots for the robots' pose and energy
        status.
        """
        super().initialize_ground_truth_listeners()
        # Ground truth topics
        topic_ns = 'ground_truth'

        # Time
        topic_info = 'time'
        msg_type = Float64
        listener_cb = self.update_time_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1000,
                                        ns_list=[[topic_ns]])

        # Communication Network
        topic_info = 'com_network'
        msg_type = String
        listener_cb = self.update_com_network_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1000,
                                        ns_list=[[topic_ns]])

    def _count_stuck_waypoints(self):
        """
        Counts the number of waypoints that produced an error but that are
        still in the robot's list of task to auction because the robot was
        not able to sell them to a teammate.
        """
        sum = 0
        for item_des in self.auctioneer_auction_to_start:
            # task_id = int(item_des.item_id.split('_')[-1])
            # TODO: caracterise stuck waypoints
            pass
        return sum

    def _check_node_events(self, node):
        """
        Checks for nodes dynamic events.
        :param node: int, the id of the node to check for dynamic events
        """
        self.get_logger().info(f"Node events {self.node_events}")
        sensors_dic = {}
        events_to_remove = []
        for event in self.node_events.copy():
            if (event["event_date"] < self.simulation_time
                    and event["event_node"] == node):
                sensors_dic[event["event_node"]] = event["sensor_types"]
                self._send_edit_node_msg(event)
                events_to_remove.append(event)
        self.nav_graph.edit_nodes_sensors(sensors_dic)
        for event in events_to_remove:
            if event in self.node_events:
                self.node_events.remove(event)

    def _send_edit_node_msg(self, event):
        """
        Checks for nodes dynamic events.
        :param event: set, a node event from the mission specs
        """
        edit_node_stp = EditNodeStamped()
        edit_node_stp.header = Header()
        edit_node_stp.header.sender_id = self.robot_id
        edit_node_stp.body_msg = EditNode()
        edit_node_stp.body_msg.discoverer_id = self.robot_id
        edit_node_stp.body_msg.simulation_time = event["event_date"]
        edit_node_stp.body_msg.node_id = event["event_node"]
        edit_node_stp.body_msg.edit_type = event["event_type"]
        edit_node_stp.body_msg.sensor_types = event["sensor_types"]
        self.send_specific_msg(edit_node_stp)

    def evaluate_team_dest(self):
        tmp_time = self.simulation_time
        robot_node = self.nav_graph.convert_pos_into_node(self.position)
        if robot_node is None:
            next_action = self.robot_plan.actions[
                self.robot_plan.actions_order[0]].goal_request
            travel_dist = (
                (self.position[0] - next_action.target_position.x)**2
                + (self.position[1] - next_action.target_position.y)**2)**.5
            tmp_time += travel_dist/self.max_speed["x"]
            robot_node = self.nav_graph.convert_pos_into_node(
                (next_action.target_position.x, next_action.target_position.y))
        for waypoint in self.waypoint_targets:
            path_dist = self.nav_graph.shortest_path_length(
                robot_node, waypoint)
            tmp_time += path_dist/self.max_speed["x"]
            robot_node = waypoint
            self.team_dest[waypoint]["date"] = tmp_time

    def can_execute(self, item, msg, verbose=False):
        """Checks if a task can be executed by the robot.
        :param item: ItemDescription
        """
        item_data = json.loads(item.item_data)
        item_sensors = item_data["sensor_list"]
        if not any(s in item_sensors for s in self.robot_sensors):
            if verbose:
                self.get_logger().info(
                    f"Dropped an auction due to sensors. Robot's sensors: "
                    f"{self.robot_sensors} ; "
                    f"Item requirements: {item_sensors} ; "
                    f"Annoucement message: {msg}")
            return False
        return True


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
