from patrol_essential.planning.utils_planning import Plan, PlanItemHandler, PlanActionHandler
from caf_messages.msg import ItemDescription

from patrol_essential.planning.patrol_sequential_planner import (
    sequential_simple_planner)
import numpy as np
import traceback


class SequentialPlanItemHandler(PlanItemHandler):
    def __init__(self, item_hdl):
        super(SequentialPlanItemHandler, self).__init__(item_hdl)
        self.actions_order = []

    def to_str_actions(self):
        res = ""
        if not self.actions_order:
            return "no actions"
        else:
            for i in range(len(self.actions_order)-1):
                res += str(i) + ": " + self.actions_order[i] + "\n\t"

    def add_action(self, action):
        super().add_action(action)
        self.actions_order.append(action.goal_id)

    def remove_action(self, action):
        super().remove_action(action)
        self.actions_order.remove(action.goal_id)

    def is_finished(self):
        if self.status == 3:
            return True

        for action_hdl in self.actions.values():
            if action_hdl.status != 2:
                return False
        self.status = 3
        return True


class SimpleSequentialPlan(Plan):
    """
    Simple sequential plan handler
    """

    def __init__(self, robot_node):
        super(SimpleSequentialPlan, self).__init__(robot_node)

        # list of actions to execute, impossible to execute concurrent actions
        self.actions_order = []

    def reset(self, threshold=100):
        """
        Resets the plan
        :param threshold: the number of items to leave in the robot's plan.
        """
        tmp_items = self.items_order.copy()
        # tmp_items.reverse()
        for item_hdl in tmp_items[threshold:]:
            # print("INSIDE", item_hdl.actions.items())
            actual_item_handler = self.p_items_hdl[item_hdl.item_hdl.item_id]
            iteration_copy = actual_item_handler.actions.copy()
            for a_id, action in iteration_copy.items():
                self.remove_action(actual_item_handler, action)

    def replan(self):
        try:
            # remove actions starting at keep_first_actions
            for action_hdl in self.actions_order:
                self.actions.pop(action_hdl)
                self.actions_order.remove(action_hdl)
            # storing an ordered list of items that are among robot's targets
            ordered_item_hdls = []
            for p_item_hdl in self.items_order:
                item_hdl = p_item_hdl.item_hdl
                ordered_item_hdls.append(item_hdl)
            # resetting items
            self.p_items_hdl = {}
            self.items_order = []
            # re adding items that are robot's targets
            for item_hdl in ordered_item_hdls:
                self.add_item(item_hdl)
            for target in self.robot_node.waypoint_targets:
                self.robot_node.get_logger().info(
                    f"Target {target}.")
            """
            self.robot_node.get_logger().info(
                f"Actions before: {self.actions}")
            for action_hdl in self.actions_order[keep_first_actions:]:
                self.actions.pop(action_hdl)
                self.actions_order.remove(action_hdl)
            for item_hdl in self.p_items_hdl:
                self.p_items_hdl[item_hdl].status = 0
            """
            self.plan_items()
            self.robot_node.get_logger().info(
                f"Actions after: {self.actions}")
            self.robot_node.get_logger().error(
                f"Replaned successfully {self}")
        except Exception as e:
            self.robot_node.get_logger().warn(
                f"Failed to replan with error {e}.")

    def add_action(self, p_item_hdl, action):
        p_item_hdl.add_action(action)
        super().add_action(action, p_item_hdl)
        self.actions_order.append(action.goal_id)

    def remove_action(self, p_item_hdl, action):
        p_item_hdl.remove_action(action)
        if not list(p_item_hdl.actions.keys()):
            item_des = ItemDescription()
            item_des.item_id = p_item_hdl.item_hdl.item_id
            item_des.item_type = p_item_hdl.item_hdl.item_type
            item_des.item_name = p_item_hdl.item_hdl.item_name
            item_des.item_data = p_item_hdl.item_hdl.item_data
            # self.robot_node.auctioneer_auction_to_start.append(item_des)
            if p_item_hdl.item_hdl.item_id in self.p_items_hdl:
                del self.p_items_hdl[p_item_hdl.item_hdl.item_id]
            else:
                self.robot_node.get_logger().warn(
                    f"Tried to remove item {p_item_hdl.item_hdl.item_id}"
                    f" from p_items_hdl: {self.p_items_hdl}")
            if p_item_hdl in self.items_order:
                self.items_order.remove(p_item_hdl)
            else:
                self.robot_node.get_logger().warn(
                    f"Tried to remove PlanItemHandler {p_item_hdl}"
                    f" from items_order: {self.items_order}")
        super().remove_action(action)
        self.actions_order.remove(action.goal_id)

    def plan_items(self):
        try:
            res = sequential_simple_planner(self.p_items_hdl, self.robot_node)
            for elem in res:
                item_hdl = elem[0]
                p_item_hdl = self.p_items_hdl[item_hdl.item_id]
                actions = elem[1]
                self.robot_node.get_logger().info(
                    f"Actions to add: {actions}"
                )
                # if actions = [], delete the plan items handler
                # if not actions:
                #     del self.p_items_hdl[item_hdl.item_id]
                for action in actions:
                    self.add_action(p_item_hdl, action)
        except Exception:
            self.robot_node.get_logger().error(traceback.format_exc())

    def add_item(self, item_hdl):
        # TODO: Temporary fix. There must be a way to dismiss the empty action
        # somewhere else.
        if item_hdl.item_type == "GoTo":
            target_position = np.array((item_hdl.target_position.x,
                                        item_hdl.target_position.y))
            robot_position = np.array((self.robot_node.position[0],
                                       self.robot_node.position[1]))
            self.robot_node.get_logger().info(f"Voici les infos dans add_item: {np.linalg.norm(target_position-robot_position), self.robot_node.waypoint_targets}")
            if (np.linalg.norm(target_position-robot_position) > 0.1
                    or self.robot_node.waypoint_targets):
                self.robot_node.get_logger().info(
                    f"est rentré dans premier if et item_id est {item_hdl.item_id}"
                    f" tandis que p_items_hdl is {self.p_items_hdl}.")
                if item_hdl.item_id not in self.p_items_hdl:
                    self.robot_node.get_logger().info("est rentré dans second if")
                    self.p_items_hdl[
                        item_hdl.item_id] = SequentialPlanItemHandler(item_hdl)
                    self.robot_node.get_logger().info(
                        f"PLANNED {item_hdl.item_id}")
                    if not self.p_items_hdl[
                            item_hdl.item_id] in self.items_order:
                        self.robot_node.get_logger().info("est rentré dans troisième if")
                        self.items_order.append(
                            self.p_items_hdl[item_hdl.item_id])
                self.robot_node.stay_and_survey = None
            elif self.robot_node.method == "DCOP":
                # Special case just to plan a waypoint on which a robot already
                # is when receiving a DCOP plan.
                self.robot_node.get_logger().info("Went through this weird if")
                self.robot_node.to_plan_later.append(item_hdl)
                self.robot_node.stay_and_survey = target_position
            else:
                self.robot_node.get_logger().info(
                    "Executed a GoTo on robot's position")
                # Add the item in the end of the to_auction list
                self.robot_node.stay_and_survey = target_position
                item_des = ItemDescription()
                item_des.item_id = item_hdl.item_id
                item_des.item_type = item_hdl.item_type
                item_des.item_name = item_hdl.item_name
                item_des.item_data = item_hdl.item_data
                """
                //!\\ Change this for SI / SSI auctions.
                """
                if self.robot_node.method == "SI":
                    self.robot_node.auctioneer_auction_to_start.append(
                        item_des)
                elif self.robot_node.method == "SSI":
                    self.robot_node.auction_scheme.auction_to_repeat.append(
                        item_des)
        else:
            self.p_items_hdl[
                item_hdl.item_id] = SequentialPlanItemHandler(item_hdl)

    def finish_action(self, action_id, execute_next_action=True):
        try:
            action_hdl = self.actions[action_id]
            action_hdl.status = 2
            action_hdl.p_item_hdl_concerned.is_finished()
            self.remove_action(action_hdl.p_item_hdl_concerned, action_hdl)
            if execute_next_action:
                self.execute_next_actions()
        except Exception:
            self.robot_node.get_logger().error(traceback.format_exc())

    def execute_next_actions(self):
        """
        Checks actions that can be executed, by example next action by
        verifying precondition if sequential, or current time if a temporal
        value is used.
        """
        self.robot_node.get_logger().info("Inside execute_next_actions")
        for action_hdl in self.actions.values():
            if action_hdl.status == 1:
                # an action is already running
                self.robot_node.get_logger().info(
                    "Returned because other action already running")
                return

        self.robot_node.get_logger().info(f"Searching in {self.actions_order}")
        for action_id in self.actions_order:
            action_hdl = self.actions[action_id]
            if action_hdl.status == 0:
                self.robot_node.get_logger().info(
                    f"Found action {action_id} that I can start")
                self.robot_node.send_goal(action_hdl.goal_request)
                action_hdl.status = 1
                p_item_hdl = action_hdl.p_item_hdl_concerned
                if not p_item_hdl.is_in_execution():
                    p_item_hdl.status = 1
                return
        if not self.actions_order:
            self.robot_node.stay_and_survey = self.robot_node.position
            self.robot_node.get_logger().info(
                f"Stay and survey {self.robot_node.position}"
                f" because actions order {self.actions_order}")
            # If using auctions, append the item to the list of tasks to auction.
            if self.robot_node.method == "SI":
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    self.robot_node.position, 0.1)          
                # Define item to sell
                item = ItemDescription()
                item.item_id = 'item_'+str(waypoint)
                item_type = 'GoTo'
                if item_type == 'GoTo':
                    item.item_name = 'goto_start_pos'
                    item.item_type = 'GoTo'
                    item.item_data = ('{"target_position": {"x": '
                                  + str(float(self.robot_node.position[0]))
                                  + ', "y": '
                                  + str(float(self.robot_node.position[1]))
                                  + ', "z": 0.0}}')
                self.robot_node.auctioneer_auction_to_start.append(item)

    def get_next_item(self):
        """
        Finds the next item to be executed by the robot according to the
        current plan.
        """
        return self.items_order[0] if self.items_order else None

    def get_next_action(self):
        """
        Finds the next action to be executed by the robot according to the
        current plan.
        """
        return self.actions_order[0] if self.actions_order else None

    def get_actions_per_item(self, p_item_hdl):
        """
        Finds actions to be executed by the robot to complete a given item.
        """
        # self.robot_node.get_logger().info(f"PlanItemHandler given as argument: {p_item_hdl, p_item_hdl.item_hdl.item_id}")
        # for g, a in self.actions.items():
            # self.robot_node.get_logger().info(f"Here I print every g, a, goal_id in get_actions_per_item: {g, a, a.p_item_hdl_concerned.item_hdl.item_id}")
        return {g: a for g, a in self.actions.items()
                if a.p_item_hdl_concerned == p_item_hdl}

    def get_last_action(self, plan_auth=True):
        try:
            if plan_auth:
                for p_item_hdl in self.p_items_hdl.values():
                    if p_item_hdl.status == 0:
                        self.plan_items()
                        continue
            if self.actions:
                return self.actions[self.actions_order[-1]].goal_request
            else:
                return None
        except Exception:
            self.robot_node.get_logger().error(traceback.format_exc())
