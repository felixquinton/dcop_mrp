import traceback
import json
import networkx as nx
import numpy as np
# from patrol_messages.msg import (TeamDestStamped, TeamDest, Header)
from caf_essential.base_node import BundleHandlerBidder
from caf_essential.auction import single_item_auction as CafSi
from caf_essential.utils import utils
from caf_messages.msg import (BidStamped, Bid, BundleDescription,
                              AnnouncementStamped, Results, ResultsStamped)

from patrol_essential.utils import utils as putils
from patrol_essential.utils import com_network as cn
from patrol_sim_robots.simulated_robot import SimulatedRobot


class PatrolSIAuction(CafSi.SingleItemAuction):
    """
    This class define the single auction scheme
    """

    def __init__(self, robot_node):
        super().__init__(robot_node)
        self.auctioneer_token = False
        self.distances_to_target = {}  # instantaneous distances, not updated

    def __init_simu_speed__(self):
        self.auction_duration = 0.5 if self.robot_node.auction_com_cost == "ours" else 0.1
        # self.robot_node.simu_refresh/4
        self.timer_period = self.auction_duration/2

    def _init_first_path(self, path):
        self.first_path = path

    def announcement_cb(self, msg, *args, **kwargs):

        self.robot_node.get_logger().info(
            f"Annoucement message from auctions,{msg}")
        # Line added from to resolve a bug CAF
        try:
            for item in msg.body_msg.item_list:
                if not self.robot_node.can_execute(item, msg):
                    return
            super().announcement_cb(msg, *args, **kwargs)
        except Exception:
            self.robot_node.get_logger().error(
                f"From annoucement_cb: {traceback.format_exc()}")

    def _find_best_bid(self, bids, verbose):
        """Finds the best bid in a list of bid messages
        :param bids: list of BidStamped ; bids to compare
        :param verbose: boolean
        """
        best_bid_val = -1e6
        best_bid_st_msg = None
        # if no one bids, auctioneer has to do the task
        best_bidder = self.robot_node.robot_id
        for bid in bids:
            if verbose:
                self.robot_node.get_logger().info(
                    "Bidder :"+str(bid.header.sender_id)
                    + " bids :"+str(bid.body_msg.bid[0]))
            bid_val = bid.body_msg.bid[0]
            if bid_val > best_bid_val:
                best_bid_val = bid_val
                best_bidder = bid.header.sender_id
                best_bid_st_msg = bid
        return best_bid_val, best_bid_st_msg, best_bidder

    def _auctioneer_bidding(self, item, announ_msg, best_bid, verbose):
        """Compute the auctioneer's bid and compare it to the best bidders'
        bid. If the auctioneer is the best bidder, builds a BidStamped msg
        and returns relevant data.
        :param item: auctioned item
        :param announ_msg: the announcement message for the auction
        :param best_bid: the best bidders' bid
        :param verbose:
        """
        waypoint_sensors = self.robot_node.nav_graph.nodes[
            int(item)]["sensors"]
        is_best = False
        auc_bid_st_msg = None
        auctioneer_bid = [None]
        # If the auctioneer has the proper sensors, compute its bid
        if any(s in waypoint_sensors for s in self.robot_node.robot_sensors):
            auctioneer_bid = self.compute_bid(
                announ_msg.body_msg.item_list[0])

            if verbose:
                self.robot_node.get_logger().info(
                    "Auctioneer :"+str(self.robot_node.robot_id)
                    + " bids :" + str(auctioneer_bid[0]))
            # Compare auctioneer's bid with the best bidders' bid
            if auctioneer_bid[0] > best_bid:
                is_best = True
                # If auctioneer is best bidder, build a BidStamped
                auc_bid_st_msg = self._build_bid_stamped(auctioneer_bid)
        return is_best, {"val": auctioneer_bid[0],
                         "msg": auc_bid_st_msg,
                         "id": self.robot_node.robot_id}

    def _build_bid_stamped(self, bid):
        """Fills a bid stamped message.
        :param bid: the bid to be associated with the bid stamped message.
        :return: BidStamped message.
        """
        st_msg = BidStamped()
        st_msg.header = utils.fill_header(self.robot_node.robot_id)
        bid_msg = Bid()
        bundle_desc = BundleDescription()
        # TODO This must be improved to ensure that bundle id are unique.
        announcement_msg = self.current_auction_auctioneer[
            'announcement_st_msg']
        bundle_id = 0
        bundle_desc.bundle_id = (self.robot_node.robot_id
                                 + '_bundle_' + str(bundle_id))
        bundle_desc.item_bundle = announcement_msg.body_msg.item_list
        bid_msg.bid = bid
        bid_msg.bundle_description = bundle_desc
        bid_msg.auction_header = \
            announcement_msg.body_msg.auction_header
        st_msg.body_msg = bid_msg
        return st_msg

    def solve_wdp(self, bids, verbose=False):
        """Basic WDP
        :param bids: bids to compare
        :param auctioneer_bid: float, the auctioneer's bid
        :return: best bidder and best bid msg stamped
        """
        best_bid_val, best_bid_st_msg, best_bidder = self._find_best_bid(
            bids, verbose)
        announcement_msg = self.current_auction_auctioneer[
            'announcement_st_msg']
        auctioned_item = \
            announcement_msg.body_msg.item_list[0].item_id.split('_')[-1]

        auc_best, auc_data = self._auctioneer_bidding(
            auctioned_item, announcement_msg, best_bid_val, verbose)
        if auc_best:
            best_bid_val, best_bid_st_msg, best_bidder = auc_data.values()

        if verbose:
            self.robot_node.get_logger().info(
                "Winner :"+str(best_bidder)
                + " with bid :"+str(best_bid_val)
                + " and message: "+str(best_bid_st_msg))
        return best_bidder, best_bid_val, best_bid_st_msg

    def compute_bid(self, item, eps=0.1):
        self.robot_node.get_logger().info(
            f"Robot position {self.robot_node.position}")
        dir_path = self.first_path + str(self.robot_node.folder_str)
        try:
            # Determining last waypoint in robot's plan
            if self.robot_node.waypoint_targets:
                robot_goal = self.robot_node.waypoint_targets[-1]
            else:
                robot_goal = None
            if robot_goal is None:
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    self.robot_node.position, eps)
            else:
                waypoint = robot_goal

            # Finding waypoint id for the item in auction
            item_data = eval(item.item_data)
            item_pos = (item_data["target_position"]["x"],
                        item_data["target_position"]["y"])
            item_node = self.robot_node.nav_graph.convert_pos_into_node(
                item_pos, eps)
            self.distances_to_target[item_node] = \
                self.robot_node.nav_graph.shortest_path_length(
                    waypoint, item_node)

            # Compute the idle cost
            has_goal = (robot_goal is not None)
            idle_cost = self.compute_idle_cost(item_node, waypoint, has_goal)

            # ener_cost = self.compute_ener_cost(item_node, waypoint)

            file_name = f"/idle_cost_{self.robot_node.robot_id}.csv"
            data_list = [self.robot_node.robot_id,
                         str(self.robot_node.simulation_time),
                         str(waypoint),
                         idle_cost]
            try:
                putils.add_row_to_csv(dir_path, file_name, data_list)
            except Exception:
                raise

            # Setting up the communication costs
            use_sheng = False
            use_ours = False
            constant = 1
            if self.robot_node.auction_com_cost == "ours":
                multiplier = 1
                use_ours = True
            elif self.robot_node.auction_com_cost == "sheng":
                multiplier = 1
                use_sheng = True
            elif self.robot_node.auction_com_cost == "no_com":
                multiplier = 0

            coms_cost = 0

            if multiplier > 0:
                if use_sheng:
                    coms_cost += self.compute_sheng_cost(
                        item_node, item_pos, waypoint)
                if use_ours:
                    coms_cost += self.compute_coms_cost(
                        item_node, item_pos, waypoint)

                file_name = f"/com_cost_{self.robot_node.robot_id}.csv"
                data_list = [self.robot_node.robot_id,
                             str(self.robot_node.simulation_time),
                             str(waypoint),
                             coms_cost]
                putils.add_row_to_csv(dir_path, file_name, data_list)

            return [float(idle_cost/(constant+idle_cost)
                    + multiplier*coms_cost)]
        except Exception:
            self.robot_node.get_logger().error("From compute_bid: "
                                               + traceback.format_exc())

    def compute_ener_cost(self, target_node, robot_node,
                          ismul=False, ispareto=False):
        """
        Computes the energy term
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :ismul: An boolean , indicating if bid computation uses multiplication
        (E*I*C) form or standard form (C*(I+E))
        :ispareto: An boolean , indicating if uses Pareto or not
        """
        energy_cons = self.robot_node.energy_cons["move"]
        if not self.robot_node.energy == 0:
            dist = self.distances_to_target[target_node]
            if dist == 0:
                return 0
            # mul_coef = (self.robot_node.nav_graph.min_weight*energy_cons)/2
            ener_cost = self.robot_node.energy/(dist*energy_cons)
            if ismul:
                ener_cost = self.robot_node.energy/dist
            if ispareto:
                ener_cost = self.robot_node.energy/(dist*energy_cons)
        else:
            ener_cost = 1e6
        return ener_cost

    def compute_reachability_cost(self, target_node, robot_node, energy_cons):
        """
        Computes a very large penalty if the robot doesn't have enougth energy
        to complete the path to the considered waypoint
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :energy_cons: The energy consumption of the robot when moving
        """
        dist = self.robot_node.nav_graph.shortest_path_length(
            robot_node, target_node)
        if dist*energy_cons <= self.robot_node.energy:
            return 0
        else:
            return 1e6

    def compute_sheng_cost(self, target_node, target_pos, robot_node):
        """
        Computes the communication maintenance term from sheng
        to complete the path to the considered waypoint
        :present_time: An integer , corresponding to current time step
        :com_network: A comGraph, modeling the communication among team members
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :robot_speed: Robot's speed
        :robots: A list of robots , the robots of the multi-robotic team
        :obstacles: A list of obstacles , obstacles in the area, defined by the
        collection attribute of class Obstacles
        :ismul: An boolean , indicating if bid computation uses multiplication
        (E*I*C) form or standard form (C*(I+E))
        :ismin: An boolean , indicating if bid computation uses the inverse
        communication term ensuring that robots stay far away from each other
        :ispareto: An boolean , indicating if uses Pareto or not
        """

        # Future date at which the robot will be at target node
        robot_last_dest_date = (self.robot_node.simulation_time)
        last_target_waypoint = robot_node
        for waypoint in self.robot_node.team_dest:
            if (self.robot_node.team_dest[waypoint]["robot_id"]
                    == self.robot_node.robot_id):
                if (self.robot_node.team_dest[waypoint]["date"]
                        > robot_last_dest_date):
                    robot_last_dest_date = \
                        self.robot_node.team_dest[waypoint]["date"]
                    last_target_waypoint = waypoint

        remaining_time = (
            self.robot_node.nav_graph.shortest_path_length(
                last_target_waypoint, target_node)
            / self.robot_node.robot_max_speed["x"]
            / self.robot_node.simu_speed)

        arrival_date = remaining_time + robot_last_dest_date

        # Robots in the same subnetwrok at auction start.
        con_comp = nx.node_connected_component(
            self.robot_node.com_graph.to_undirected(),
            self.robot_node.robot_id.split('_')[-1])

        # Predicting the poses of teammates at arrival date.
        futures_poses = {}
        for robot_id in self.robot_node.robots_spec_data.keys():
            futures_poses[robot_id] = self._predict_future_pos(
                robot_id, arrival_date)["future_pos"]
        futures_poses[self.robot_node.robot_id] = target_pos

        dist_list = []
        for robot in futures_poses:
            id = robot.split('_')[-1]
            if id in con_comp and robot != self.robot_node.robot_id:
                # if robot != self.robot_node.robot_id:
                tmp_pos = futures_poses[robot]
                dist = ((tmp_pos[0]-target_pos[0])**2
                        + (tmp_pos[1]-target_pos[1])**2)**.5
                dist_list.append(dist)

        dist_list.sort()
        res = 0
        alpha_cpt = 0
        alpha = 0.5
        rc = self.robot_node.robot_spec_data["com_range"]
        for d in dist_list:
            res += alpha**alpha_cpt*np.exp(-d/rc)
            alpha_cpt += 1
        return res

    def compute_coms_cost(self, target_node, target_pos, robot_node,
                          ismul=False, ismin=False, ispareto=False):
        """
        Computes the communication maintenance term
        to complete the path to the considered waypoint
        :present_time: An integer , corresponding to current time step
        :com_network: A comGraph, modeling the communication among team members
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :robot_speed: Robot's speed
        :robots: A list of robots , the robots of the multi-robotic team
        :obstacles: A list of obstacles , obstacles in the area, defined by the
        collection attribute of class Obstacles
        :ismul: An boolean , indicating if bid computation uses multiplication
        (E*I*C) form or standard form (C*(I+E))
        :ismin: An boolean , indicating if bid computation uses the inverse
        communication term ensuring that robots stay far away from each other
        :ispareto: An boolean , indicating if uses Pareto or not
        """
        H = self.robot_node.com_graph.to_undirected()
        for edge in list(H.edges):
            w = H.edges[edge]["weight"]
            if w == 1:
                H.remove_edge(*edge)

        nb_con_nodes_start = len(nx.node_connected_component(
            H, self.robot_node.robot_id.split('_')[-1]))

        # Future date at which the robot will be at target node
        robot_last_dest_date = (self.robot_node.simulation_time)
        last_target_waypoint = robot_node
        for waypoint in self.robot_node.team_dest:
            if (self.robot_node.team_dest[waypoint]["robot_id"]
                    == self.robot_node.robot_id):
                if (self.robot_node.team_dest[waypoint]["date"]
                        > robot_last_dest_date):
                    robot_last_dest_date = \
                        self.robot_node.team_dest[waypoint]["date"]
                    last_target_waypoint = waypoint

        remaining_time = (
            self.robot_node.nav_graph.shortest_path_length(
                last_target_waypoint, target_node)
            / self.robot_node.robot_max_speed["x"]
            / self.robot_node.simu_speed)

        arrival_date = remaining_time + robot_last_dest_date

        # Predicting the poses of teammates at arrival date.
        futures_poses = {}
        tmp_simulated_robots = {}
        try:
            for robot_id in self.robot_node.robots_spec_data.keys():
                predicted_pose = self._predict_future_pos(
                    robot_id, arrival_date)
                futures_poses[robot_id] = predicted_pose["future_pos"]
                tmp_simulated_robots[robot_id] = SimulatedRobot(
                    robot_id, self.robot_node,
                    position=[float(predicted_pose["future_pos"][0]),
                              float(predicted_pose["future_pos"][1])],
                    full_init=False)
                if predicted_pose["before_target"] is None:
                    tmp_simulated_robots[robot_id].edge = None
                    tmp_simulated_robots[robot_id].node = \
                        predicted_pose["after_target"]
                elif predicted_pose["after_target"] is None:
                    tmp_simulated_robots[robot_id].edge = None
                    tmp_simulated_robots[robot_id].node = \
                        predicted_pose["before_target"]
                else:
                    tmp_simulated_robots[robot_id].edge = (
                        predicted_pose["before_target"],
                        predicted_pose["after_target"])
                    tmp_simulated_robots[robot_id].node = None
            futures_poses[self.robot_node.robot_id] = target_pos
            tmp_simulated_robots[self.robot_node.robot_id].edge = None
            tmp_simulated_robots[self.robot_node.robot_id].node = target_node
        except Exception as e:
            raise e

        # Generating a temporary network graph with the predicted poses.
        """
        future_network = self._build_tmp_network(futures_poses)

        nb_con_nodes_future = len(nx.node_connected_component(
            future_network, self.robot_node.robot_id.split('_')[-1]))
        """
        future_network = cn.comGraph(
            self.robot_node.robots_spec_data, self.robot_node.proba_obs_data)
        future_network.update_edges(tmp_simulated_robots, logging=False)

        J = future_network.to_undirected()
        for edge in list(J.edges):
            w = J.edges[edge]["weight"]
            if w == 1:
                J.remove_edge(*edge)

        nb_con_nodes_future = len(nx.node_connected_component(
            J, self.robot_node.robot_id.split('_')[-1]))
        return (nb_con_nodes_future/nb_con_nodes_start - 1) - int(self.robot_node.is_initial_auctioneer)

    def _build_tmp_network(self, futures_poses):
        """
        //!!\\ Attention, ne tiens compte que de la distance. Pour prendre en
        compte les obstacles, il faudra recoder.
        """
        future_network = nx.Graph()
        for robot_id in futures_poses.keys():
            future_network.add_node(int(robot_id.split('_')[-1]))
        for robot_id, robot_fut_node in futures_poses.items():
            robot_fut_pos = np.array(robot_fut_node)
            robot_id_int = robot_id.split('_')[-1]
            robot_com_range = self.robot_node.robots_spec_data[
                robot_id]["com_range"]
            for teammate_id, teammate_fut_node in futures_poses.items():
                teammate_fut_pos = np.array(teammate_fut_node)
                teammate_id_int = teammate_id.split('_')[-1]
                hop_dist = np.linalg.norm(robot_fut_pos-teammate_fut_pos)
                teammate_com_range = self.robot_node.robots_spec_data[
                    teammate_id]["com_range"]
                if (hop_dist <= robot_com_range
                        and hop_dist <= teammate_com_range):
                    future_network.add_edges_from(
                        [(robot_id_int, teammate_id_int)])
        return future_network

    def _predict_future_pos(self, robot_id, future_date):
        future_pos = self.robot_node.teammate_poses[robot_id]
        # I need the target that is just before future_date and the one that
        # is just after
        before_target_date = self.robot_node.simulation_time
        after_target_date = self.robot_node.simulation_time + 1e6
        before_target = None
        after_target = None
        for w, dic in self.robot_node.team_dest.items():
            if dic["robot_id"] == robot_id:
                if (dic["date"] > before_target_date
                        and dic["date"] <= future_date):
                    before_target_date = dic["date"]
                    before_target = w
                if (dic["date"] < after_target_date
                        and dic["date"] > future_date):
                    after_target_date = dic["date"]
                    after_target = w
        if before_target is not None:
            future_pos = self.robot_node.nav_graph.nodes[
                before_target]["pos"]
        if after_target is not None:
            future_pos = self.robot_node.nav_graph.nodes[
                after_target]["pos"]
        if before_target is not None and after_target is not None:
            future_pos = self._determine_position(
                before_target, after_target, before_target_date,
                after_target_date, future_date)

        return {"future_pos": future_pos,
                "before_target": before_target,
                "after_target": after_target}

    def _determine_position(self, w1, w2, d1, d2, current_date):
        try:
            dic_waypoints = {w1: d1, w2: d2}
            path = self.robot_node.nav_graph.shortest_path(w1, w2)
            rolling_date = d1
            rolling_waypoint = w1
            for w in path[1: -1]:
                dic_waypoints[w] = (
                    rolling_date
                    + (self.robot_node.nav_graph.shortest_path_length(
                        rolling_waypoint, w)
                       / self.robot_node.robot_max_speed["x"]
                       / self.robot_node.simu_speed))
                rolling_date = dic_waypoints[w]
                rolling_waypoint = w

            before_target_date = self.robot_node.simulation_time
            after_target_date = self.robot_node.simulation_time + 1e6
            before_target = None
            after_target = None
            for w, d in dic_waypoints.items():
                if (d > before_target_date and d <= current_date):
                    before_target_date = d
                    before_target = w
                if (d < after_target_date and d > current_date):
                    after_target_date = d
                    after_target = w
            pos_dic = nx.get_node_attributes(self.robot_node.nav_graph, "pos")
            pos_bt = pos_dic[before_target]
            pos_at = pos_dic[after_target]
            d_ab = ((pos_at[0]-pos_bt[0])**2+(pos_at[1]-pos_bt[1])**2)**.5
            travel_time_ab = (d_ab
                              / self.robot_node.robot_max_speed["x"]
                              / self.robot_node.simu_speed)

            x_current = pos_bt[0] + (pos_at[0]-pos_bt[0])*(
                (current_date - dic_waypoints[before_target])
                / travel_time_ab)
            y_current = pos_bt[1] + (pos_at[1]-pos_bt[1])*(
                (current_date - dic_waypoints[before_target])
                / travel_time_ab)
        except Exception:
            self.robot_node.get_logger().error("From determine_position: "
                                               + traceback.format_exc())
        return [int(x_current), int(y_current)]

    def compute_idle_cost(self, target_node, robot_node,
                          has_goal=False, ismul=False):
        """
        Computes the waypoint's idleness term. The formula is
        (ETA to target_node - target_node last visit date)/(travel time).
        And we perform the following computations:
        - ETA to target_node = ETA to last node in robot's plan
                               + travel time from last node to target_node.
        - travel time = ETA to target_node - present time.
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :has_goal: boolean, true if robot has a non empty plan
        :ismul: An boolean , indicating if bid computation uses multiplication
        (E*I*C) form or standard form (C*(I+E))
        """
        # distance from robot's destination to target waypoint.
        dist = self.distances_to_target[target_node]
        # Travel time from robot's destination to target waypoint.
        t_travel = dist * self.robot_node.max_speed["x"]
        t = self.robot_node.simulation_time
        # ETA of the robot to its destination (last waypoint of its path)
        if has_goal:
            t_dest = self.robot_node.team_dest[target_node]["date"]
        else:
            # If robot has no plan, it is available immediatly.
            t_dest = t
        # if present_time > 0:
        # team_dest_date = self.robot_node.team_dest[target_node]["date"]
        # ETA of robot on target waypoint
        eta = t_dest + t_travel
        # Last visit date of target waypoint.
        t_last = self.robot_node.last_visits[target_node]
        if not dist == 0:
            idle_cost = 1 + (t - t_last)/(eta - t)
        else:
            # If robot is already on the waypoint and has an empty plan
            # or has non empty plan ending on the waypoint.
            # Both cases are handle thanks to the small 1e-6 that prevent
            # the denominator from being 0.
            idle_cost = 1 + (t - t_last)/(eta - t + 1e-6)
        return idle_cost

    def results_cb(self, msg, *args, **kwargs):
        """
        Results callback
        :param msg: received message
        """
        # self.robot_node.get_logger().info(
        #     "Result message from auctions,"+str(msg))
        # Line added from CAF to resolve a bug
        if msg.header.sender_id == self.robot_node.robot_id:
            return

        # Check if concerned by award
        results_msg = msg.body_msg
        for award in results_msg.award_list:
            if award.bidder_id == self.robot_node.robot_id:

                self.robot_node.get_logger().info(
                    'Received an award ' + '\n'
                    + str(award))
                    
                # Send acknowledgement to auctioneer
                self.send_acknowledgement(msg, award.bundle_id)

                # Reset current auction tour status
                self.robot_node.current_auction_bidder = None

                # Adding new waypoint to target list
                item_pos = json.loads(results_msg.bundle_descriptions[
                    0].item_bundle[0].item_data)["target_position"]
                new_target_id = \
                    self.robot_node.nav_graph.convert_pos_into_node((
                        item_pos["x"], item_pos["y"]))
                self.robot_node.waypoint_targets.append(new_target_id)

                dir_path = self.first_path + str(self.robot_node.folder_str)
                file_name = "/allocation.csv"
                data_list = [str(self.robot_node.simulation_time),
                             str(self.robot_node.robot_id),
                             str(new_target_id)]
                putils.add_row_to_csv(dir_path, file_name, data_list)

                target_arrival = (
                    self.distances_to_target[new_target_id]
                    / self.robot_node.robot_max_speed["x"]
                    / self.robot_node.simu_speed)
                robot_last_dest_date = self.robot_node.simulation_time
                robot_last_dest_waypoint = \
                    self.robot_node.nav_graph.convert_pos_into_node(
                        self.robot_node.position, 1e-3)
                for waypoint in self.robot_node.team_dest:
                    if (self.robot_node.team_dest[waypoint]["robot_id"]
                            == self.robot_node.robot_id):
                        if (self.robot_node.team_dest[waypoint]["date"]
                                > robot_last_dest_date):
                            robot_last_dest_date = \
                                self.robot_node.team_dest[waypoint]["date"]
                            robot_last_dest_waypoint = waypoint

                # Travel time from robot's pos to target node
                target_arrival = (
                    self.robot_node.nav_graph.shortest_path_length(
                        robot_last_dest_waypoint, new_target_id)
                    / self.robot_node.robot_max_speed["x"]
                    / self.robot_node.simu_speed)

                self.robot_node.team_dest[new_target_id]["date"] = (
                    target_arrival + robot_last_dest_date)
                self.robot_node.team_dest[
                    new_target_id]["robot_id"] = self.robot_node.robot_id
        if not self.robot_node.robot_plan.actions_order:
            if self.robot_node.waypoint_targets:
                self.robot_node.plan_waypoints_by_id(
                    self.robot_node.waypoint_targets[0])

    def auctioneer_self_award(self, msg, best_bid_val, bundle_id,
                              verbose=True):
        results_msg = msg.body_msg
        for award in results_msg.award_list:
            self.robot_node.get_logger().info(f"Self-awarding \n {award}")


        try:
            # Adding new waypoint to waypoint targets
            results_msg = msg.body_msg
            item_pos = json.loads(results_msg.bundle_descriptions[
                0].item_bundle[0].item_data)["target_position"]
            new_target_id = \
                self.robot_node.nav_graph.convert_pos_into_node((
                    item_pos["x"], item_pos["y"]))

            self.robot_node.waypoint_targets.append(new_target_id)

            dir_path = self.first_path + str(self.robot_node.folder_str)
            file_name = "/allocation.csv"
            data_list = [str(self.robot_node.simulation_time),
                         str(self.robot_node.robot_id),
                         str(new_target_id)]
            putils.add_row_to_csv(dir_path, file_name, data_list)

            # Finding the last target of the robot given current assignment
            robot_last_dest_date = self.robot_node.simulation_time

            robot_last_dest_waypoint = \
                self.robot_node.nav_graph.convert_pos_into_node(
                    self.robot_node.position, 1e-3)
            for waypoint in self.robot_node.team_dest:
                if (self.robot_node.team_dest[waypoint]["robot_id"]
                        == self.robot_node.robot_id):
                    if (self.robot_node.team_dest[waypoint]["date"]
                            > robot_last_dest_date):
                        robot_last_dest_date = \
                            self.robot_node.team_dest[waypoint]["date"]
                        robot_last_dest_waypoint = waypoint
            if verbose:
                self.robot_node.get_logger().info(
                    f"Team dest from self award {self.robot_node.team_dest}")

            # Travel time from robot's pos to target node
            # TODO: replace this call with the dict computed in preprocessing
            target_arrival = (
                self.robot_node.nav_graph.shortest_path_length(
                    robot_last_dest_waypoint, new_target_id)
                / self.robot_node.robot_max_speed["x"]
                / self.robot_node.simu_speed)

            # Updating team dest with estimated arrival date
            self.robot_node.team_dest[new_target_id]["date"] = (
                target_arrival + robot_last_dest_date)

            self.robot_node.team_dest[
                new_target_id]["robot_id"] = self.robot_node.robot_id
            if verbose:
                self.robot_node.get_logger().info(
                    f"""Successfuly updated the team dest. New team_dest:
                        {self.robot_node.team_dest}""")

            self.robot_node.get_logger().info("Finished self award.")
            if not self.robot_node.robot_plan.actions_order:
                if self.robot_node.waypoint_targets:
                    self.robot_node.plan_waypoints_by_id(
                        self.robot_node.waypoint_targets[0])
        except Exception:
            self.robot_node.get_logger().error("From self award: "
                                               + traceback.format_exc())

    def awaiting_acknowledgements_timer_cb(self):
        """
        Callback use to check award acknowledgement
        """
        self.auction_awaiting_acknowledgements['timer'].cancel()
        # Verify all awards have been accepted
        for bundle_id in self.auction_awaiting_acknowledgements[
                'allocated_bundles_to_confirm'].keys():
            elem = self.auction_awaiting_acknowledgements[
                'allocated_bundles_to_confirm'][bundle_id]
            if not elem['is_accepted']:
                #  TODO Do something if not accepted
                pass

        # Delete auction from awaiting auction
        self.robot_node.passed_auctions_auctioneer.append(
            self.auction_awaiting_acknowledgements)
        self.auction_awaiting_acknowledgements = None

        # Start new SI auction
        self.robot_node.publish_auction_termination()

        # self.send_announcement(marker="awaiter")

    def _no_valid_bids(self, forbidden_task_pos):
        self.robot_node.get_logger().info(
            f"Inside no valid bids {forbidden_task_pos}")
        try:
            # Results
            results_msg = Results()
            results_msg.award_list = []
            # ResultsStamped
            msg = ResultsStamped()
            msg.header = utils.fill_header(self.robot_node.robot_id)
            msg.body_msg = results_msg
        except Exception:
            self.robot_node.get_logger().error(
                "Error while filling dummy Results message.")

        try:
            # Store auction to wait acknowledgements
            self.current_auction_auctioneer['bid_opening'] = False
            self.auction_awaiting_acknowledgements = \
                self.current_auction_auctioneer
            self.current_auction_auctioneer = None
            self.auction_awaiting_acknowledgements[
                'allocated_bundles_to_confirm'] = {}

            # Send results
            self.robot_node.send_msg_to_teammates(msg)

            self.robot_node.get_logger().info("Empty results sent")
        except Exception:
            self.robot_node.get_logger().error(
                "Error while asking for acknowledgements on dummy Results.")

        # Timer to wait awards acknowledgements
        timer_period = self.timer_period  # seconds
        self.auction_awaiting_acknowledgements['timer'] = \
            self.robot_node.create_timer(
                timer_period,
                self.awaiting_acknowledgements_timer_cb)

        if forbidden_task_pos == "last":
            try:
                if self.robot_node.current_auctioned_task.item_id not in [
                        obj.item_id for obj in
                        self.robot_node.auctioneer_auction_to_start]:
                    self.robot_node.auctioneer_auction_to_start.append(
                        self.robot_node.current_auctioned_task)

                    dir_path = self.first_path + str(
                        self.robot_node.folder_str)
                    file_name = "/auction_to_start_log.csv"
                    data_list = [self.robot_node.simulation_time,
                                 self.robot_node.robot_id,
                                 "no_valid_bids",
                                 self.robot_node.auctioneer_auction_to_start]
                    putils.add_row_to_csv(dir_path, file_name, data_list)

                self.robot_node.current_auctioned_task = None

                # self.robot_node.auctioneer_auction_to_start.pop(0)
                # self.robot_node.publish_auction_termination()
                self.robot_node.get_logger().info("Bid was None")
            except Exception:
                self.robot_node.get_logger().warn(
                    "From no valid bids: " + traceback.format_exc())
        elif forbidden_task_pos == "first":
            # Doing nothing is equivalent to remove the task and put it
            # again at the start.
            pass
        else:
            # Otherwise, TBD
            pass

    def _fill_item_data(self, item):
        item_data = json.loads(item.item_data)
        p = list(item_data["target_position"].values())[:2]
        waypoint_id = self.robot_node.nav_graph.convert_pos_into_node(p)
        item_data["sensor_list"] = self.robot_node.nav_graph.nodes[
            waypoint_id]["sensors"]
        return json.dumps(item_data)

    def send_announcement(self, marker="unknown"):
        """
        Initiates a SI auction.
        """
        try:
            self.robot_node.get_logger().info("Running send_annoucenement")
            item = self.robot_node.auctioneer_auction_to_start[0]
            item.item_data = self._fill_item_data(item)
            # Create announcement stamped msg
            msg = AnnouncementStamped()
            # fill stamp
            msg.header = utils.fill_header(self.robot_node.robot_id)
            # fill auction header
            msg.body_msg.auction_header.auction_id = \
                'auction_'+self.robot_node.robot_id+'_'+str(
                    self.robot_node.auction_cpt)
            # Define deadline
            auction_duration = self.auction_duration  # seconds (int)
            msg.body_msg.auction_deadline = \
                self.robot_node.get_clock().now().to_msg()
            msg.body_msg.auction_deadline.nanosec += int(auction_duration*1e9)
            msg.body_msg.item_list = [item]
            self.robot_node.get_logger().info(
                f"Announcement msg with sensors {msg}")
            # Store this auction for robot as "auctioneer" role
            self.current_auction_auctioneer = {
                'announcement_st_msg': msg, 'bids_st_msgs': [],
                'bid_opening': True}
            # publish msg
            self.robot_node.send_auction_msg(msg)

            # Create timer to end auction
            self.current_auction_auctioneer['timer'] = \
                self.robot_node.create_timer(
                auction_duration,
                self.current_auction_auctioneer_timer_cb,
                callback_group=self.robot_node.callback_group)

            dir_path = self.first_path + str(self.robot_node.folder_str)
            file_name = "/annoucements_log.csv"
            data_list = [self.robot_node.simulation_time,
                         self.robot_node.robot_id,
                         msg.body_msg.item_list]
            putils.add_row_to_csv(dir_path, file_name, data_list)
        except Exception:
            self.robot_node.get_logger().error(traceback.format_exc())
