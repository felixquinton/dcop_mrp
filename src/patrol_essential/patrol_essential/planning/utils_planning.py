# TODO replaces this dependency ! ( using import_module ?)
from patrol_essential.utils.items_handlers import return_item_handler
import traceback


class PlanActionHandler:
    """
    Class to manage Action planned to execute, use a ROS2 Action Goal Request
    as attribute
    """

    def __init__(self, goal_request, p_item_hdl_concerned):
        self.goal_id = goal_request.goal_id
        self.goal_request = goal_request
        self.status = 0
        self.p_item_hdl_concerned = p_item_hdl_concerned


class PlanItemHandler:
    """
    We do not use directly ItemHandler class because it's more convenient to
    implement another class to switch execution mode. by example sequential
    or concurrent.
    """

    def __init__(self, item_hdl):
        self.item_hdl = item_hdl
        self.status = 0  # To Plan
        self.actions = {}

    def to_str(self):
        res = self.item_hdl.item_id + ":\n" + \
            "status : \n\t" + str(self.status) + "\n" + \
            "actions : \n\t" + \
            str(self.to_str_actions())
        return res

    def to_str_actions(self):
        return self.actions.keys()

    def add_action(self, action):
        self.actions[action.goal_id] = PlanActionHandler(action, self)

    def remove_action(self, action):
        self.actions.pop(action.goal_id)

    def is_in_execution(self):
        if self.status != 0:
            return True
        for action_hdl in self.actions.values():
            if action_hdl.status != 0:
                return True
        return False


class Plan:
    """
    Class that manages the robot plan, i.e. all items (and actions) done, to
    do and in execution since the beginning of the mission.
    """

    def __init__(self, robot_node):
        self.robot_node = robot_node  # robot_node owning the Plan
        self.p_items_hdl = {}  # dict of planned plan_item_hdl (key: item_id,
        # value: plan_item_hdl)
        self.actions = {}  # dict of planned actions
        self.items_order = []  # list ordering items

    def add_item(self, item_hdl):
        self.p_items_hdl[item_hdl.item_id] = PlanItemHandler(item_hdl)
        self.items_order.append(self.p_items_hdl[item_hdl.item_id])

    def add_action(self, action, p_item_hdl_concerned):
        self.actions[action.goal_id] = PlanActionHandler(
            action, p_item_hdl_concerned)

    def remove_action(self, action):
        self.actions.pop(action.goal_id)

    def print_plan(self):
        res = ""
        for p_item_hdl in self.p_items_hdl.values():
            res += p_item_hdl.to_str()
        return res

    def add_items_from_bundle_hdl(self, bundle_hdl):
        try:
            for item_description_msg in bundle_hdl.bundle_description.item_bundle:
                item_hdl = return_item_handler(item_description_msg)
                self.add_item(item_hdl)
        except Exception:
            self.robot_node.get_logger().error(traceback.format_exc())

    def plan_items(self):
        pass

    def execute_next_actions(self):
        """
        Checks actions that can be executed, by example next action by
        verifying precondition if sequential, or current
        time if a temporal value is used.
        """
        pass
