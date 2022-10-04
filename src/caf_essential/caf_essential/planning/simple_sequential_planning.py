from caf_essential.planning.utils_planning import Plan, PlanItemHandler

from caf_essential.planning.caf_sequential_planner import sequential_simple_planner
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

        # list of actions to execute, it is impossible to execute concurrent actions
        self.actions_order = []

    def add_action(self, p_item_hdl, action):
        p_item_hdl.add_action(action)
        super().add_action(action, p_item_hdl)
        self.actions_order.append(action.goal_id)

    def remove_action(self, p_item_hdl, action):
        p_item_hdl.remove_action(action)
        super().remove_action(action)
        self.actions_order.remove(action.goal_id)

    def plan_items(self):
        try:
            res = sequential_simple_planner(self.p_items_hdl, self.robot_node)
            for elem in res:
                item_hdl = elem[0]
                p_item_hdl = self.p_items_hdl[item_hdl.item_id]
                actions = elem[1]
                # if actions = [], delete the plan items handler
                # if not actions:
                #     del self.p_items_hdl[item_hdl.item_id]
                for action in actions:
                    self.add_action(p_item_hdl, action)
        except:
            self.robot_node.get_logger().error(traceback.format_exc())

    def add_item(self, item_hdl):
        # TODO: Temporary fix. There must be a way to dismiss the empty action
        # somewhere else.
        if item_hdl.item_type == "GoTo":
            target_position = np.array((item_hdl.target_position.x,
                                        item_hdl.target_position.y))
            robot_position = np.array((self.robot_node.position[0],
                                       self.robot_node.position[1]))
            if np.linalg.norm(target_position-robot_position) > 1e-3:
                self.p_items_hdl[
                    item_hdl.item_id] = SequentialPlanItemHandler(item_hdl)
            else:
                self.robot_node.get_logger().info(
                    "Executed a GoTo on robot's position")
        else:
            self.p_items_hdl[
                item_hdl.item_id] = SequentialPlanItemHandler(item_hdl)

    def finish_action(self, action_id):
        try:
            action_hdl = self.actions[action_id]
            action_hdl.status = 2
            action_hdl.p_item_hdl_concerned.is_finished()
            self.execute_next_actions()
        except:
            self.robot_node.get_logger().error(traceback.format_exc())

    def execute_next_actions(self):
        """
        Checks actions that can be executed, by example next action by
        verifying precondition if sequential, or current time if a temporal
        value is used.
        """
        for action_hdl in self.actions.values():
            if action_hdl.status == 1:
                # an action is already running
                return

        for action_id in self.actions_order:
            action_hdl = self.actions[action_id]
            if action_hdl.status == 0:
                self.robot_node.send_goal(action_hdl.goal_request)
                action_hdl.status = 1
                p_item_hdl = action_hdl.p_item_hdl_concerned
                if not p_item_hdl.is_in_execution():
                    p_item_hdl.status = 1
                return

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
        except:
            self.robot_node.get_logger().error(traceback.format_exc())
