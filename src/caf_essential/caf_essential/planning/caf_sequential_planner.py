import uuid
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
    except:
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
    else:
        last_waypoint = robot_node.nav_graph.convert_pos_into_node(
            robot_node.position, eps)
    path = robot_node.shortest_paths[last_waypoint][goal_waypoint][1:]

    for wp_num in path:
        wp = robot_node.nav_graph.nodes[wp_num]
        target_position = Point(x=float(wp['pos'][0]), y=float(wp['pos'][1]))
        actions.append(GoTo.Goal(goal_id=str(uuid.uuid4()),
                                 target_position=target_position))
    return actions
