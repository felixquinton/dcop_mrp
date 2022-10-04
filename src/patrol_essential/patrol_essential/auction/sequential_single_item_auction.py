import math
import json
import traceback
import numpy as np
import networkx as nx
from caf_essential.auction import sequential_single_item_auction as baseSSI
from caf_essential.utils import utils
from caf_essential.base_node import BundleHandlerBidder
from patrol_messages.msg import TeamDestStamped, TeamDest, Header
from caf_messages.msg import (BidStamped, Bid, BundleDescription)


class SSIAuction(baseSSI.SSIAuction):
    """
    This class define the single auction scheme
    """

    def __init__(self, robot_node):
        super().__init__(robot_node)
        self.auctioneer_token = False
        self.distances_to_target = {}  # instantaneous distances, not updated
        self.auction_to_repeat = []

    def solve_wdp(self, bids):
        """
        Basic WDP
        :param bids: bids to compare
        :param auctioneer_bid: float, the auctioneer's bid
        :return: best bidder and best bid msg stamped
        """
        best_bid_val = -1e6
        best_bid_st_msg = None
        # if no one bids, auctioneer has to do the task
        best_bidder = self.robot_node.robot_id
        for bid in bids:
            print("Bidder :", bid.header.sender_id,
                  " bids :", bid.body_msg.bid[0])
            bid_val = bid.body_msg.bid[0]
            if bid_val > best_bid_val:
                best_bid_val = bid_val
                best_bidder = bid.header.sender_id
                best_bid_st_msg = bid

        announcement_msg = self.current_auction_auctioneer[
            'announcement_st_msg']
        auctioneer_bid = self.compute_bid(
            announcement_msg.body_msg.item_list[0])

        print("Auctioneer :", self.robot_node.robot_id,
              " bids :", auctioneer_bid[0])
        if auctioneer_bid[0] > best_bid_val:
            best_bidder = self.robot_node.robot_id
            best_bid_val = auctioneer_bid[0]
            best_bid_st_msg = BidStamped()
            best_bid_st_msg.header = utils.fill_header(
                self.robot_node.robot_id)
            bid_msg = Bid()
            bundle_desc = BundleDescription()
            # TODO This must be improved to ensure that bundle id are unique.
            announcement_msg = self.current_auction_auctioneer[
                'announcement_st_msg']
            bundle_id = 0
            bundle_desc.bundle_id = self.robot_node.robot_id + '_bundle_' \
                + str(bundle_id)
            # The rest is ok.
            bundle_desc.item_bundle = announcement_msg.body_msg.item_list
            # Computing bid
            bid_msg.bid = auctioneer_bid
            bid_msg.bundle_description = bundle_desc
            bid_msg.auction_header = announcement_msg.body_msg.auction_header
            best_bid_st_msg.body_msg = bid_msg
        print("Winner :", best_bidder, " with bid :", best_bid_val)
        return best_bidder, best_bid_val, best_bid_st_msg

    def compute_bid(self, item, eps=0.1):
        try:
            item_data = eval(item.item_data)
            item_pos = (item_data["target_position"]["x"],
                        item_data["target_position"]["y"])
            if not len(self.robot_node.robot_plan.actions) == 0:
                # TODO exploit directly last planned state ?
                last_action = self.robot_node.robot_plan.get_last_action()
                last_pos = {"x": last_action.target_position.x,
                            "y": last_action.target_position.y}
                robot_goal = last_pos
            else:
                robot_goal = None

            if robot_goal is None:
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    self.robot_node.position, eps)
                remaining_distance = 0
            else:
                # TODO Ensure that this doesnt cause issues when robot's path
                # has several nodes.
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    (robot_goal['x'], robot_goal['y']), eps)
                remaining_distance = (
                    (robot_goal['x']-self.robot_node.position[0])**2
                    + (robot_goal['y']-self.robot_node.position[1])**2)**.5
            item_node = self.robot_node.nav_graph.convert_pos_into_node(
                item_pos, eps)

            self.distances_to_target[item_node] = \
                self.robot_node.nav_graph.shortest_path_length(
                    waypoint, item_node) + remaining_distance

            idle_cost = self.compute_idle_cost(item_node, waypoint)

            ener_cost = self.compute_ener_cost(item_node, waypoint)

            coms_cost = self.compute_coms_cost(item_node, item_pos, waypoint)

            # Now use navGraph methods to compute the robot -> task distance.
            # bid = (self.distances_to_target[item_node] + remaining_distance)
            return [float(idle_cost + ener_cost + coms_cost)]
        except:
            self.robot_node.get_logger().error(traceback.format_exc())

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
        # print("HERE IS THE ENERGY COST")
        # print(ener_cost)
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

    def compute_coms_cost(self, target_node, target_pos,
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

        nb_con_nodes_start = len(nx.node_connected_component(
            self.robot_node.com_graph.to_undirected(),
            self.robot_node.robot_id.split('_')[-1]))

        # Future date at which the robot will be at target node
        remaining_time = math.ceil(
            self.distances_to_target[target_node]
            / self.robot_node.robot_max_speed["x"])
        robot_last_dest_date = \
            self.robot_node.get_clock().now().to_msg().sec
        for waypoint in self.robot_node.team_dest:
            if (self.robot_node.team_dest[waypoint]["robot_id"]
                    == self.robot_node.robot_id):
                if (self.robot_node.team_dest[waypoint]["date"]
                        > robot_last_dest_date):
                    robot_last_dest_date = \
                        self.robot_node.team_dest[waypoint]["date"]
        arrival_date = remaining_time + robot_last_dest_date

        # Predicting the poses of teammates at arrival date.
        futures_poses = {}
        for robot_id in self.robot_node.robots_spec_data.keys():
            futures_poses[robot_id] = self._predict_future_pos(
                robot_id, arrival_date)
        futures_poses[self.robot_node.robot_id] = target_pos

        # print("FUTURE POSES", futures_poses)
        # print("TEAM DEST", self.robot_node.team_dest)

        # Generating a temporary network graph with the predicted poses.
        future_network = self._build_tmp_network(futures_poses)

        nb_con_nodes_future = len(nx.node_connected_component(
            future_network, self.robot_node.robot_id.split('_')[-1]))

        return nb_con_nodes_future/nb_con_nodes_start - 1

    def _build_tmp_network(self, futures_poses):
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
        before_target_date = self.robot_node.get_clock().now().to_msg().sec
        after_target_date = self.robot_node.get_clock().now().to_msg().sec+1e6
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
            future_pos = self.robot_node.mission_spec_data[
                "waypoint_"+str(before_target)]["pos"]
        if after_target is not None:
            future_pos = self.robot_node.mission_spec_data[
                "waypoint_"+str(after_target)]["pos"]
        return future_pos

    def compute_idle_cost(self, target_node, robot_node, ismul=False):
        """
        Computes the waypoint's idleness term
        to complete the path to the considered waypoint
        :target_node: The node of the graph that is evaluated
        :robot_node: The node of the robot
        :ismul: An boolean , indicating if bid computation uses multiplication
        (E*I*C) form or standard form (C*(I+E))
        """
        dist = self.distances_to_target[target_node]
        present_time = self.robot_node.get_clock().now().to_msg().sec
        # if present_time > 0:
        team_dest_date = self.robot_node.team_dest[target_node]["date"]
        if not dist == 0:
            if present_time >= team_dest_date:
                idle_cost = (
                    1
                    + (present_time - self.robot_node.last_visits[target_node])
                    / dist * self.robot_node.max_speed["x"])
                if ismul:
                    idle_cost = (
                        dist*self.robot_node.max_speed["x"]
                        + present_time
                        - self.robot_node.last_visits[target_node])
            else:
                idle_cost = abs(
                    1
                    + (present_time - team_dest_date)
                    / dist * self.robot_node.max_speed["x"])
                if ismul:
                    idle_cost = (dist * self.robot_node.max_speed["x"]
                                 - team_dest_date)
        else:
            idle_cost = (1 + present_time
                         - self.robot_node.last_visits[target_node])
        return idle_cost

    def results_cb(self, msg, *args, **kwargs):
        """
        Results callback
        :param msg: received message
        """

        if msg.header.sender_id == self.robot_node.robot_id:
            return

        super().results_cb(msg, *args, **kwargs)

        # Check if concerned by award
        results_msg = msg.body_msg
        for award in results_msg.award_list:
            if award.bidder_id == self.robot_node.robot_id:
                item_pos = json.loads(results_msg.bundle_descriptions[
                    0].item_bundle[0].item_data)["target_position"]
                new_target_id = \
                    self.robot_node.nav_graph.convert_pos_into_node((
                        item_pos["x"], item_pos["y"]))
                self.robot_node.waypoint_targets.append(new_target_id)
                target_arrival = math.ceil(
                    self.distances_to_target[new_target_id]
                    / self.robot_node.robot_max_speed["x"])
                robot_last_dest_date = \
                    self.robot_node.get_clock().now().to_msg().sec
                for waypoint in self.robot_node.team_dest:
                    if (self.robot_node.team_dest[waypoint]["robot_id"]
                            == self.robot_node.robot_id):
                        if (self.robot_node.team_dest[waypoint]["date"]
                                > robot_last_dest_date):
                            robot_last_dest_date = \
                                self.robot_node.team_dest[waypoint]["date"]
                self.robot_node.team_dest[new_target_id]["date"] = (
                    target_arrival + robot_last_dest_date)
                self.robot_node.team_dest[
                    new_target_id]["robot_id"] = self.robot_node.robot_id

        if results_msg.close_round:
            # A déterminer avec une enchère
            self.robot_node.auction_scheme.auctioneer_token = True

    def announcement_cb(self, msg, *args, **kwargs):
        if msg.header.sender_id == self.robot_node.robot_id:
            return
        self.robot_node.auction_scheme.auctioneer_token = False
        super().announcement_cb(msg, *args, **kwargs)

    def auctioneer_self_award(self, msg, best_bid_val, bundle_id):
        results_msg = msg.body_msg
        for award in results_msg.award_list:
            self.robot_node.get_logger().info(
                'Self-awarding ' + '\n'
                + str(award))

        # Store award in order to plan it
        try:
            announcement_msg = \
                self.auction_awaiting_acknowledgements[
                    'announcement_st_msg'].body_msg
            bundle_description = BundleDescription()
            bundle_description.item_bundle = announcement_msg.item_list
            bundle_description.bundle_id = bundle_id
            self_bid_msg = Bid(auction_header=announcement_msg.auction_header,
                               bundle_description=bundle_description,
                               bid=[best_bid_val])
            bundle_hdl = BundleHandlerBidder(self_bid_msg)
            self.robot_node.obtained_bundles[award.bundle_id] = bundle_hdl
            self.robot_node.robot_plan.add_items_from_bundle_hdl(bundle_hdl)
            self.robot_node.robot_plan.plan_items()
        except TypeError:
            self.robot_node.get_logger().error(
              'ERROR, type error on self.current_auction_bidder dict')
            raise
        except KeyError:
            self.robot_node.get_logger().error(
              'ERROR, key error on self.current_auction_bidder dict')
            raise

        item_pos = json.loads(results_msg.bundle_descriptions[
            0].item_bundle[0].item_data)["target_position"]
        new_target_id = \
            self.robot_node.nav_graph.convert_pos_into_node((
                item_pos["x"], item_pos["y"]))
        self.robot_node.waypoint_targets.append(new_target_id)
        # Travel time from robot's pos to target node
        target_arrival = math.ceil(
            self.distances_to_target[new_target_id]
            / self.robot_node.robot_max_speed["x"])
        # Finding the last target of the robot given current assignment
        robot_last_dest_date = \
            self.robot_node.get_clock().now().to_msg().sec
        for waypoint in self.robot_node.team_dest:
            if (self.robot_node.team_dest[waypoint]["robot_id"]
                    == self.robot_node.robot_id):
                if (self.robot_node.team_dest[waypoint]["date"]
                        > robot_last_dest_date):
                    robot_last_dest_date = \
                        self.robot_node.team_dest[waypoint]["date"]
        # Updating team dest with estimated arrival date
        self.robot_node.team_dest[new_target_id]["date"] = (
            target_arrival + robot_last_dest_date)
        self.robot_node.team_dest[
            new_target_id]["robot_id"] = self.robot_node.robot_id

        # create teaTeamDest
        team_dest_msg = TeamDest()
        team_dest_msg.robot_id = self.robot_node.robot_id
        team_dest_msg.team_dest = json.dumps(
            self.robot_node.team_dest)
        team_dest_stp = TeamDestStamped()
        team_dest_stp.header = Header()
        team_dest_stp.header.sender_id = self.robot_node.robot_id
        team_dest_stp.header.required_acknowledgement = False
        team_dest_stp.body_msg = team_dest_msg

        self.robot_node.send_msg_to_teammates(team_dest_stp)

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
        self.robot_node.auctioneer_auction_to_start.pop(0)
        self.robot_node.auction_cpt += 1
