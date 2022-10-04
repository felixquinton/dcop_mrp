import uuid
import networkx as nx
from caf_messages.action import GoTo, AbstractAction
from geometry_msgs.msg import Point
import traceback


def sequential_simple_planner(p_items_hdl, robot_node):
    """
    Simple function that plans sequential actions to execute items
    :param p_items_hdl:
    :param robot_node:
    :return:
    """
    try:
        res = []
        for p_item_hdl in p_items_hdl.values():
            # if status not equal to 0 therefore the item was already planned
            if p_item_hdl.status != 0:
                continue
            item_hdl = p_item_hdl.item_hdl
            if item_hdl.item_type == "GoTo":
                res.append([item_hdl, plan_goto_item(item_hdl, robot_node)])
                # Set status to not 0 to express that the item is planned.
                p_item_hdl.status = .5
            elif item_hdl.item_type == "AbstractAction":
                res.append([item_hdl, AbstractAction.Goal(
                    goal_id=str(uuid.uuid4()),
                    target_duration=item_hdl.target_duration)])
        return res
    except Exception:
        robot_node.get_logger().error(traceback.format_exc())


def plan_goto_item(item_hdl, robot_node, eps=0.1):
    actions = []
    goal_pos = (item_hdl.target_position.x, item_hdl.target_position.y)
    goal_waypoint = robot_node.nav_graph.convert_pos_into_node(goal_pos, eps)
    last_action = robot_node.robot_plan.get_last_action(plan_auth=False)
    if last_action:
        last_pos = {"x": last_action.target_position.x,
                    "y": last_action.target_position.y}
        last_waypoint = robot_node.nav_graph.convert_pos_into_node(
            (last_pos['x'], last_pos['y']), eps)
        robot_node.get_logger().info(f"Has a last action, pos is {last_pos} and waypoint is {last_waypoint}.")
    else:
        last_waypoint = robot_node.nav_graph.convert_pos_into_node(
            robot_node.position, eps)
        robot_node.get_logger().info(f"Has no last action, position is {robot_node.position} waypoint is {last_waypoint}.")
    try:
        path = robot_node.nav_graph.shortest_path(
            last_waypoint, goal_waypoint)[1:]
    except:
        path = []
        robot_node.get_logger().error(
            f"Failed to find {last_waypoint} or {goal_waypoint} in graph."
            f"Last action {last_action}")
    for wp_num in path:
        wp = robot_node.nav_graph.nodes[wp_num]
        target_position = Point(x=float(wp['pos'][0]), y=float(wp['pos'][1]))
        actions.append(GoTo.Goal(goal_id=str(uuid.uuid4()),
                                 target_position=target_position))

    return actions
