import rclpy
import json
import subprocess
import os
import time
import math
import numpy as np
import networkx as nx
import traceback


from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from patrol_messages.msg import SendTour
from rlb_utils.msg import Goal as RLBGoal

from caf_sim_robots.sim_robots_function import SimRobots as BaseSim
from patrol_sim_robots.simulated_robot import SimulatedRobot
from patrol_essential.auction.navigation_graph import navGraph
from patrol_essential.utils import com_network as cn
from patrol_essential import token_dispenser_class as tk
from patrol_essential.dcop import dcop_utils

from patrol_essential.utils import utils

from patrol_messages.msg import (EditEdgeStamped, EditNodeStamped)


class SimRobots(BaseSim):
    """
    This is the simulation node for patrol. This node do several things:
    - publish ground truth
    - use simulated robots classes to manage robots information
    - call a step function with a given time increment to update system states
    This node rely on SimulatedRobot class to call their corresponding step
    function at each run of the simulation.
    """

    def __init__(self):

        self.solving_dcop_flag = False
        self.launch_dcop = False

        super().__init__(init_sim_robots=False, init_simulation_timer=False)

        communication_range, scenario_path = self._declare_parameters()

        if self.method not in ["SI", "DCOP"]:
            self.get_logger().error(
                f"Unsupported solving method: {self.method}")
            rclpy.shutdown()
        self.get_logger().info(
            f"Setting up robot with solving method {self.method}")

        # A few parameters if using "DCOP"
        # A quick and dirty way to ensure that a DCOP is launch right away.
        self.first_alloc = (self.method == "DCOP")
        # Have the simulation work step by step to catch CN splitting
        self.step_length = 0.1
        self.step_wise = True if self.method == "SLOW_DCOP" else False

        self.awaiting_goal_request = []

        # self.simulation_timer.cancel()

        self.last_auction_start = self.simulation_time
        self.last_auction_end = self.simulation_time
        self.time_of_reset = time.time()

        self.auction_period = 5.0

        # These parameters help to determine when to reset the simu timer.
        self.awaiting_for_auction = False
        self.awaiting_for_goto = False

        self.team_members_data = utils.get_team_members_data(self)
        self.robots_spec_data = utils.get_robots_spec_data(self)
        self.mission_spec_data = utils.get_mission_spec_data(self)

        # Keep track of the unfeasible tasks that must be reallocated in DCOPs
        self.tasks_to_reallocate = {
            r_name.split('_')[-1]: [] for r_name in self.robots_spec_data
        }
        self.tried_to_reallocate = {}

        self.time_factor = self.mission_spec_data["time_factor"]
        self.simulation_timer_period = self.mission_spec_data["simu_period"]

        self.simulation_timer_period = 0.2

        self.robots_list = self.team_members_data['team_members_list']

        if scenario_path:
            data_path = os.path.join(
                get_package_share_directory('patrol_mission_spec'), 'data',
                scenario_path)
            with open(data_path) as datafile:
                data = json.loads(json.load(datafile))

            self.navigation_graph = navGraph(
                utils.import_generated_nav_graph(data))

            for robot, robot_data in self.robots_spec_data.items():
                robot_data["com_range"] = communication_range
            self.robots_com_range = {
                r_id: communication_range for r_id in self.robots_spec_data}

            # Init com network
            self.com_network = cn.comGraph(
                self.robots_spec_data, data["edge_to_edge_com"])
        else:
            self.navigation_graph = navGraph(self.mission_spec_data["graph"])
            self.robots_com_range = {
                r_id: self.robots_spec_data[r_id]["com_range"]
                for r_id in self.robots_spec_data}

            # Init com network
            self.com_network = cn.comGraph(
                self.robots_spec_data,
                self.robots_spec_data["robot_1"]["com_prob_dic"])

        # Keep track of the allocation to consider the right tasks in DCOPs
        self.r_allocation = {
            r_name.split('_')[-1]: [] for r_name in self.robots_spec_data}
        self.r_allocation['1'] = [
            w for w in self.navigation_graph.nodes]

        # Init space graph
        self.shortest_paths = dict(nx.shortest_path_length(
            self.navigation_graph, weight='weight'))

        # Schedule of edge related events
        self.edge_events = self.mission_spec_data["edge_events"]

        # Init simulated robots
        self.simulated_robots = {}
        self.initialize_simulated_robots()

        # Saving the mission's parameters
        if not os.path.exists(self.first_path+str(self.folder_str)):
            os.makedirs(self.first_path+str(self.folder_str))
            log_file = (
                self.first_path+str(self.folder_str) + "/obstacle_probas.txt")
            with open(log_file, mode='a+') as outfile:
                json.dump(
                    self.robots_spec_data["robot_1"]["com_prob_dic"], outfile)

        # Init last visit dates
        self.last_visits = {}

        log_file = (
            self.first_path+str(self.folder_str) + "/robot_specs.json")
        with open(log_file, mode='a+') as outfile:
            json.dump(
                self.robots_spec_data, outfile)
        log_file = (
            self.first_path+str(self.folder_str) + "/mission_specs.json")
        with open(log_file, mode='a+') as outfile:
            json.dump(
                self.mission_spec_data, outfile)

        for waypoint in self.navigation_graph.nodes:
            self.last_visits[waypoint] = self.simulation_time

        self.token_dispenser = tk.TokenDispenser(self)

        # tsp = traveling_salesman_problem(self.navigation_graph, "greedy")
        # distance = nx.classes.function.path_weight(
        #     self.navigation_graph, tsp, 'weight')

        self.simulation_timer = self.create_timer(
            self.simulation_timer_period, self.run_simulation,
            callback_group=self.callback_group)

        cmd = ("ros2 bag record /comm_auctions/announcement "
               '/comm_auctions/award_acknowledgement /comm_auctions/bid '
               '/comm_patrol/last_visits /comm_patrol/team_dest '
               '/comm_auctions/results '
               f"-o {self.first_path+str(self.folder_str)}/bag")

        if self.method in ["SI"]:
            subprocess.Popen(cmd, shell=True)

        self.last_clock_date = time.time()

    def _declare_parameters(self):
        self.declare_parameter("communication_range", 100)
        communication_range = self.get_parameter("communication_range").value

        self.declare_parameter("scenario_path", "")
        scenario_path = self.get_parameter("scenario_path").value

        self.declare_parameter("folder")
        self.first_path = self.get_parameter("folder").value + "/"

        self.declare_parameter("method", "SI")
        self.method = self.get_parameter("method").value

        self.declare_parameter("auction_com_cost", "")
        self.auction_com_cost = self.get_parameter("auction_com_cost").value

        self.declare_parameter("scenario_id")
        self.folder_str = self.get_parameter("scenario_id").value

        return (communication_range, scenario_path)

    def _init_probas(self, mul, ones=False, data_dic=None):
        if data_dic is None:
            data_dic = self.robots_spec_data["robot_1"]["com_prob_dic"]
        for key, item in data_dic.items():
            real_obs_prob = 1 - (1 - data_dic[key])*mul
            if np.random.rand() > real_obs_prob and not ones:
                data_dic[key] = 0
            else:
                data_dic[key] = 1

    def initialize_simulated_robots(self):
        """
        This function creates class to handle CAF simulated robots
        NEEDS to be surcharged with your use case (and so you SimulatedRobot
        classes)
        """
        pos = [200.0, 200.0]  # [119.91915745825732, 203.15187127554503]  #  [200.0, 200.0]   # [0.0, 0.0]
        for robot_id in self.robots_list:
            self.simulated_robots[robot_id] = SimulatedRobot(
                robot_id, self, position=pos)

    def initialize_listeners(self):
        super().initialize_listeners()
        self.initialize_auction_termination_listeners()
        self.initialize_goto_request_termination_listeners()
        self.initialize_dynamic_events_listeners()

    def initialize_publishers(self):
        super().initialize_publishers()
        self.initialize_event_publishers()
        self.initialize_dcop_plans_publishers()
        self.initialize_rlb_publishers()

    def initialize_ground_truth_publishers(self):
        super().initialize_ground_truth_publishers()
        topic_ns = 'ground_truth'
        ns_list = []
        for robot_id in self.robots_list:
            ns_list.append([robot_id, topic_ns])

        # Time
        topic_info = 'time'
        msg_type = Float64
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers, queue_size=100,
                                         ns_list=ns_list)
        # Idleness
        topic_info = 'idleness'
        msg_type = String
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers, queue_size=100,
                                         ns_list=ns_list)

    def initialize_event_publishers(self):
        topic_ns = 'event'
        ns_list = []
        for robot_id in self.robots_list:
            ns_list.append([robot_id, topic_ns])

        # Sends a ping to the robots to indicate they must share data
        topic_info = 'share_data'
        msg_type = String
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=100, ns_list=ns_list)

        # Sends a ping to a robot to give it the auctioneer token
        topic_info = 'auctioneer_token'
        msg_type = String
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=100, ns_list=ns_list)

    def initialize_dcop_plans_publishers(self):
        topic_ns = 'dcop'
        ns_list = []
        for robot_id in self.robots_list:
            ns_list.append([robot_id, topic_ns])

        # Sends plans computed with pyDCOP to the robots
        topic_info = 'dcop_plan'
        msg_type = SendTour
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=100, ns_list=ns_list)

    def initialize_rlb_publishers(self):
        topic_ns = 'rlb'
        ns_list = [['sim_node_publisher', topic_ns]]

        # Sends plans goals to RobotLabBridge through dedicated .msg
        topic_info = 'targets'
        msg_type = RLBGoal
        self.create_and_store_publishers(
            topic_info, msg_type, self.node_publishers,
            queue_size=100, ns_list=ns_list)

    def initialize_auction_termination_listeners(self):
        """
        Initialize listeners to receive the messages indicating that an auction
        is finished (including the acknowledgement).
        """
        # Token topic
        topic_ns = 'auction_termination'

        # Resquests
        topic_info = 'reset_auction_timer'
        msg_type = String
        listener_cb = self.receive_auction_termination_msgs
        self.create_subscription(
            msg_type, '/'+topic_ns+'/'+topic_info,
            listener_cb, 100)

    def initialize_goto_request_termination_listeners(self):
        """
        Initialize listeners to receive the messages indicating that a new goto
        goal is requested or that a robot has no further goto goal.
        """
        # Token topic
        topic_ns = 'goto_goal_request'

        # Resquests
        topic_info = 'reset_timer'
        msg_type = String
        listener_cb = self.receive_goto_request_termination_msgs
        self.create_subscription(
            msg_type, '/'+topic_ns+'/'+topic_info,
            listener_cb, 100)

    def initialize_dynamic_events_listeners(self):
        # BEWARE, the '/' at beginning is mandatory to avoid putting the topic
        # in the node namespace
        comm_topic_ns = 'comm_events'
        # General edit edge topic
        topic_info = 'edit_edge'
        msg_type = EditEdgeStamped
        listener_cb = self.receive_edit_edge
        self.create_subscription(
            msg_type, '/'+comm_topic_ns+'/'+topic_info,
            listener_cb, 100)
        # General edit node topic
        topic_info = 'edit_node'
        msg_type = EditNodeStamped
        listener_cb = self.receive_edit_node
        self.create_subscription(
            msg_type, '/'+comm_topic_ns+'/'+topic_info,
            listener_cb, 100)

    def receive_edit_edge(self, msg):
        self.get_logger().info(f"Sim node received edit edge {msg}")

    def receive_edit_node(self, msg):
        if self.method == "DCOP":
            self.simulation_timer.cancel()
        # self.get_logger().info(f"Sim node received edit node {msg}")
        self.navigation_graph.edit_nodes_sensors(
            {msg.body_msg.node_id: msg.body_msg.sensor_types})
        if (self.method == "DCOP"
                and msg.body_msg.discoverer_id == msg.header.sender_id):
            self.get_logger().info(
                f"Launched a DCOP solve after an event on node {msg.body_msg.node_id}"
                f"discovered by robot {msg.body_msg.discoverer_id}")
            self._awaiting_dcop_allocation(msg.body_msg.discoverer_id)

    def receive_auction_termination_msgs(self, msg):
        """
        Update data when an auction termination message was received.
        """
        self.last_auction_end = self.simulation_time
        if self.awaiting_for_auction:
            self.simulation_timer.reset()
            # self.awaiting_for_auction = False
            self.time_of_reset = time.time()
            self.stopped_for_auction = time.time() - self.stopped_for_auction
            self.get_logger().info("Reset timer after an auction.")

    def receive_goto_request_termination_msgs(self, msg):
        """
        Update data when a GoTo goal is properly closed and the following
        action is requested.
        Also reset the simulation timer if the simulation waits for goto
        request (i.e. the current step is an 'arrival').
        """
        # self.get_logger().info(
        #     f"Received goto request termination msg {msg}.")
        robot_id = msg.data
        if robot_id in self.awaiting_goal_request:
            # self.get_logger().warn(
            #     f"""Removing robot {robot_id} from
            #     {self.awaiting_goal_request}.""")
            self.awaiting_goal_request.remove(robot_id)

        if self.awaiting_for_goto and not self.awaiting_goal_request:
            if not self.launch_dcop:
                # self.get_logger().info("Reset timer")
                self.simulation_timer.reset()
            else:
                self.get_logger().info(
                    "Did not reset timer in goto request termination to wait "
                    "for a DCOP.")
            # self.awaiting_for_goto = False
            # self.get_logger().info(
            #     "Reset simu timer from receive goto term in simu node")

    def publish_event_msgs(self):
        topic_ns = 'event'
        for robot_id in self.robots_list:

            # Event based msg
            topic_info = 'share_data'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = String(data="share_data")
            self.publish_msg(msg, topic_keys)

        # Second for loop so that auction tokens are sent after shared data
        if not self.method == "DCOP":
            for robot_id in self.robots_list:
                if self.token_dispenser.fifo_list:
                    next_auctioneer = self.token_dispenser.fifo_list[0]
                    if next_auctioneer == robot_id:
                        distrib_msg = String()
                        distrib_msg.data = next_auctioneer
                        topic_info = 'auctioneer_token'
                        topic_keys = [robot_id, topic_ns, topic_info]
                        self.publish_msg(distrib_msg, topic_keys)
                        self.last_auction_start = self.simulation_time
                        self.get_logger().info(
                            "Stopped simu during an auction for robot."
                            + str(next_auctioneer))
                        self.stopped_for_auction = time.time()
        else:
            self._awaiting_dcop_allocation()

    def publish_simulation_time(self, auto_reset=False):
        topic_ns = 'ground_truth'
        for robot_id in self.robots_list:
            # Time
            topic_info = 'time'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = Float64(data=float(self.simulation_time))
            self.publish_msg(msg, topic_keys)

    def publish_ground_truth(self, auto_reset=False):
        topic_ns = 'ground_truth'
        robots_ener = {}
        for robot_id in self.robots_list:
            simulated_robot = self.simulated_robots[robot_id]

            # Pose
            topic_info = 'pose'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = Pose(position=simulated_robot.position)
            self.publish_msg(msg, topic_keys)

            # Energy
            topic_info = 'energy'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = Float64(data=float(simulated_robot.energy))
            self.publish_msg(msg, topic_keys)
            robots_ener[simulated_robot.robot_id] = float(
                simulated_robot.energy)

            # Idleness
            topic_info = 'idleness'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = String(data=json.dumps(self.last_visits))
            self.publish_msg(msg, topic_keys)

            # Communication Network
            topic_info = 'com_network'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = String(data=self.com_network.convert_to_str())
            self.publish_msg(msg, topic_keys)

    def find_com_edges(self):
        res = []
        com_range = next(iter(self.robots_spec_data.values()))["com_range"]
        for e1 in self.navigation_graph.edges():
            a1 = self.navigation_graph.nodes[e1[0]]['pos']
            a2 = self.navigation_graph.nodes[e1[1]]['pos']
            for e2 in self.navigation_graph.edges():
                b1 = self.navigation_graph.nodes[e2[0]]['pos']
                b2 = self.navigation_graph.nodes[e2[1]]['pos']
                if not e1 == e2 and not a1 == a2 and not b1 == b2:
                    dist = min([self._euclidian_dist(a1, b1),
                                self._euclidian_dist(a2, b2),
                                self._euclidian_dist(a1, b2),
                                self._euclidian_dist(a2, b1)])
                    if dist < com_range and (e2, e1) not in res:
                        res.append((e1, e2))
        return res

    def _euclidian_dist(self, l1, l2):
        return ((l1[0]-l2[0])**2+(l1[1]-l2[1])**2)**.5

    def _find_next_increment(self):
        if self.first_alloc:
            self.first_alloc = False
            return ("dcop", 1.0)
        arrival_dists = {}
        arrival_planned = False
        for robot in self.simulated_robots.values():
            if len(robot.actions_to_update) > 0:
                dic = robot.actions_to_update
                pos = robot.position
                speed = robot.max_speed['x']
                arrival_dist = dic[next(iter(dic))].dist_to_finish(pos)/speed
                arrival_dists[robot.robot_id] = arrival_dist
        if arrival_dists:
            soonest_arrival = min(arrival_dists.values())
            arrival_planned = True
        else:
            soonest_arrival = 1e6
        next_auction_date = max((self.auction_period
                                 + self.last_auction_start
                                 - self.simulation_time), 0)
        if soonest_arrival < next_auction_date:
            event_type = "arrival"
            event_time = soonest_arrival
            for robot, dist in arrival_dists.items():
                if (math.isclose(dist, soonest_arrival)
                        and robot not in self.awaiting_goal_request):
                    self.awaiting_goal_request.append(robot)
        else:
            event_type = "auction"
            event_time = next_auction_date
            # if its time to start an auction but the fifo list is empty, reset
            # time before next auction
            if not self.token_dispenser.fifo_list:
                event_type = "arrival"
                event_time = soonest_arrival
                self.last_auction_start = self.simulation_time

        if not arrival_planned and not self.token_dispenser.fifo_list:
            return ("none", 0)
        if event_time > self.step_length and self.step_wise:
            return ("step", self.step_length)
        return (event_type, event_time)

    def _remove_robots_without_goal_from_awaiting_resquest_list(self):
        for robot in self.simulated_robots.values():
            if (len(robot.actions_to_update) == 0
                    and robot.robot_id in self.awaiting_goal_request):
                self.awaiting_goal_request.remove(robot.robot_id)
                # self.get_logger().info(
                #     f"""Removed {robot.robot_id} cause it had no goals.
                #     It had goals {robot.actions_to_update}""")

    def _awaiting_goto_request_timer(self):
        # self._remove_robots_without_goal_from_awaiting_resquest_list()
        if not self.awaiting_goal_request:
            self.simulation_timer.reset()
            # self.awaiting_for_goto = False
            self.time_of_reset = time.time()
            self.await_goto_requests.cancel()

    def _awaiting_dcop_allocation(self, r_id=None, verbose=False):
        self.simulation_timer.cancel()
        start_time = time.time()
        self.solving_dcop_flag = True
        self.com_network.remove_integer_nodes(len(self.robots_list))
        components = [
            list(c)
            for c in nx.strongly_connected_components(self.com_network)]
        if r_id is not None:
            if isinstance(r_id, str):
                r_id = r_id.split('_')[-1]
            components = [c for c in components if r_id in c]
        for component in components:
            self.get_logger().info(
                f"Tasks to reallocate {self.tasks_to_reallocate}")
            tasks = np.concatenate(
                [ts for r, ts in self.r_allocation.items() if r in component])
            for r, ts in self.tasks_to_reallocate.items():
                if r in component:
                    for t in ts:
                        if t not in tasks:
                            tasks = np.concatenate([tasks, ts])

            tasks = np.unique(np.int16(tasks))
            self.get_logger().warn(
                f"Starting DCOP procedure with tasks: {tasks}")

            stdout, is_trivial, unfeasible_tasks, pre_assignment, pre_assigned_tasks = \
                dcop_utils.solve_dcop(
                    component,
                    tasks,
                    {int(r.split("_")[-1])-1: d["robot_sensors"]
                     for r, d in self.robots_spec_data.items()},
                    {w: s for w, s in nx.get_node_attributes(
                        self.navigation_graph, "sensors").items()},
                    np.array(list(nx.get_node_attributes(
                        self.navigation_graph, "pos").values())),
                    self.navigation_graph,
                    sim_node=self)
            for r in component:
                for w in self.tasks_to_reallocate[r]:
                    if w not in unfeasible_tasks:
                        self.tasks_to_reallocate[r].remove(w)
                for w in unfeasible_tasks:
                    if tasks[w] not in self.tasks_to_reallocate[r]:
                        self.tasks_to_reallocate[r].append(tasks[w])
                        self.tried_to_reallocate[tasks[w]] = self.tried_to_reallocate.get(
                            tasks[w], []) + [r for r in component]
            tasks = np.delete(tasks, unfeasible_tasks)
            tasks = np.setdiff1d(tasks, list(pre_assigned_tasks.keys()))
            assignment = dcop_utils.get_assignment_from_pydcop(
                self, stdout, component, tasks, pre_assigned_tasks, is_trivial)
            tours = dcop_utils.compute_tours(
                self.navigation_graph, assignment, self)
            for r in tours:
                self.r_allocation[str(r)] = tours[r]
            self.send_tours(tours)
        # A dirty way to wait until the robots have processed the DCOP results
        dir = f"{self.first_path}{self.folder_str}"
        file_name = "/computing_times.csv"
        timer = time.time() - start_time
        data_list = [self.simulation_time,
                     timer]
        try:
            utils.add_row_to_csv(dir, file_name, data_list)
        except Exception:
            raise
        time.sleep(3)
        self.launch_dcop = False
        self.solving_dcop_flag = False
        self.simulation_timer.reset()
        if verbose:
            self.get_logger().info(f"DCOP stdout {stdout}.")
            self.get_logger().info(f"Tours: {tours}")
            self.get_logger().info(f"r allocation: {self.r_allocation}")

    def send_tours(self, tours):
        """Send planned tours (sequence of waypoints) to the robot nodes.
        :param tours: dic, with keys being the robots' ids and values, the
         tours as lists of waypoint ids.
        """
        topic_ns = 'dcop'
        for robot_id in self.robots_list:
            # Event based msg
            topic_info = 'dcop_plan'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = SendTour()
            json_serializable_tours = {}
            for r in tours:
                json_serializable_tours[int(r)] = [int(w) for w in tours[r]]
            msg.tours = json.dumps(json_serializable_tours)
            self.publish_msg(msg, topic_keys)
            self.get_logger().info(
                f"Published DCOP msg {msg} on topic {topic_keys}")

    def publish_target_to_lrb(self, robot_id, goal_request):
        rlb_goal_msg = RLBGoal()
        rlb_goal_msg.robot_id = robot_id
        rlb_goal_msg.goal_sequence_id = goal_request.goal_id
        rlb_goal_msg.sequence = [goal_request.target_position]
        self.publish_msg(rlb_goal_msg,
                         ['sim_node_publisher', 'rlb', 'targets'])

    def _check_dcop_realloc(self):
        components = [
            list(c)
            for c in nx.strongly_connected_components(self.com_network)]
        for c in components:
            for r1 in c:
                for r2 in c:
                    if r1 != r2:
                        for w in self.tasks_to_reallocate[r1]:
                            if r2 not in self.tried_to_reallocate[w]:
                                self.get_logger().info(
                                    f"I returned True and {r1} because {r2, w, self.tasks_to_reallocate, self.tried_to_reallocate}")
                                return (True, r1)
        return (False, None)

    def run_simulation(self, verbose=False, display_clock=False):
        """
        This function runs the simulation.
        """
        # return
        self.simulation_timer.cancel()
        if self.solving_dcop_flag:
            return
        with self.lock:

            event_type, event_time = self._find_next_increment()
            self.get_logger().info(
                f"Found event {event_type}, at time {event_time}")

            if event_type == "auction":
                self.awaiting_for_auction = True
                self.awaiting_for_goto = False
            elif event_type == "arrival":
                self.awaiting_for_auction = False
                self.awaiting_for_goto = True

            if verbose:
                self.get_logger().info("---------------------------------")
                self.get_logger().info(
                    f"Time since last reset {time.time()-self.time_of_reset}")
                self.get_logger().info(
                    f"Awaiting request from {self.awaiting_goal_request}")
                self.get_logger().info("---------------------------------")
                self.get_logger().info("Temps avant le prochain event.")
                self.get_logger().info(f"{event_type}, {event_time}")
                self.get_logger().info("---------------------------------")
                self.get_logger().info("---------------------------------")
            # self.get_logger().info(
            #     "Position des robots au temps :", self.simulation_time)

                for robot in self.simulated_robots.values():
                    self.get_logger().info(
                        str(robot.robot_id)+" : "+str(robot.position))
                self.get_logger().info("---------------------------------")

            self.last_clock_date = time.time()

            # increment = self.time_factor*self.simulation_timer_period
            increment = event_time
            self.simulation_time += increment
            if display_clock:
                self.get_logger().info("---------------------------------")
                self.get_logger().info(
                    f"Simulation time : {self.simulation_time}")
                self.get_logger().info("---------------------------------")

            self.publish_simulation_time()

            if self.simulation_time > 200000:
                for robot in self.simulated_robots.values():
                    for handlers in robot.robot_actions_servers.values():
                        for handler in handlers.values():
                            try:
                                handler.server.destroy()
                            except Exception:
                                self.get_logger().error(traceback.format_exc())
                # self.executor.shutdown(1)
                rclpy.shutdown()
                # sys.exit()
                return

            for robot in self.simulated_robots.values():
                robot.step(increment)

            if verbose:
                self.get_logger().info("---------------------------------")
                self.get_logger().info(
                    f"Position des robots au temps : {self.simulation_time}")

                for robot in self.simulated_robots.values():
                    self.get_logger().info(
                        f"{robot.robot_id} {robot.position}")
                self.get_logger().info("---------------------------------")

            modified = self.com_network.update_edges(self.simulated_robots)
            if modified:
                if not os.path.exists(self.first_path+str(self.folder_str)):
                    os.makedirs(self.first_path+str(self.folder_str))
                log_file = (
                    f"{self.first_path}{self.folder_str}"
                    "/com_graph.csv")
                self.com_network.log(self.simulation_time, log_file)
                self.launch_dcop, dcop_starter = self._check_dcop_realloc()

            self.publish_ground_truth()
            if event_type == "dcop":
                time.sleep(0.5)  # Just ensuring that the robots have process
                # their pose if a dcop is coming.
            if event_type in ("auction", "dcop"):
                self.publish_event_msgs()
            elif event_type in ("none", "step"):
                self.get_logger().info(f"J'ai step avec {event_type}")
                self.simulation_timer.reset()
            # self.get_logger().info("Finished a simulation iteration.")
            if self.launch_dcop:
                self.get_logger().info(
                    f"Started a DCOP reallocation from robot {dcop_starter}.")
                self._awaiting_dcop_allocation(dcop_starter)


def main(args=None):
    rclpy.init(args=args)

    node = SimRobots()

    executor = MultiThreadedExecutor(num_threads=100)
    rclpy.spin(node, executor)
    """

    while rclpy.ok() and node.simulation_time < 20:
        event_type, event_time = node._find_next_increment()
        print("Found", event_type)

        if event_type == "auction":
            node.awaiting_for_auction = True
            node.awaiting_for_goto = False
        elif event_type == "arrival":
            node.awaiting_for_auction = False
            node.awaiting_for_goto = True

        node.last_clock_date = time.time()

        increment = event_time
        node.simulation_time += increment

        print("Simulation time", node.simulation_time)

        node.publish_simulation_time()

        for robot in node.simulated_robots.values():
            robot.step(increment)

        modified = node.com_network.update_edges(node.simulated_robots)
        if modified:
            if not os.path.exists(node.first_path+str(node.folder_str)):
                os.makedirs(node.first_path+str(node.folder_str))
            log_file = (
                f"{node.first_path}{node.folder_str}"
                "/com_graph.csv")
            node.com_network.log(node.simulation_time, log_file)

        node.publish_ground_truth()
        if event_type == "dcop":
            time.sleep(0.5)  # Just ensuring that the robots have process
            # their pose if a dcop is coming.
        if event_type in ("auction", "dcop"):
            node.publish_event_msgs()
            while node.awaiting_for_auction:
                rclpy.spin_once(node)
                print(node.awaiting_for_auction)
                time.sleep(0.01)
        elif event_type == "arrival":
            while node.awaiting_for_goto:
                rclpy.spin_once(node)
                time.sleep(0.01)
        elif event_type in ("none", "step"):
            rclpy.spin_once(node)
            time.sleep(0.01)
    """

    print("Outside spin")

    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
