from caf_sim_robots.simulated_robot import ActionHandler
from caf_sim_robots.simulated_robot import SimulatedRobot as BaseSimulatedRobot
from rclpy.action import GoalResponse
from caf_essential.utils import utils
from patrol_essential.utils.utils import euclidean_distance_2d
from caf_messages.action import GoTo
import traceback
from action_tutorials_interfaces.action import Fibonacci
from geometry_msgs.msg import Point


class GoToHandler(ActionHandler):
    """
    Class to handle CAF GoTo goal.
    """

    def __init__(self, goal_request):
        super(GoToHandler, self).__init__(goal_request)

        self.target_position = goal_request.target_position
        self.current_travel_distance = 0.0

    def step(self, increment, simulated_robot=None, *args, **kwargs):
        """
        Step function to increment execution duration.
        :param increment: time increment to use
        :param simulated_robot: a simulated robot object - robot executing the
        goal
        """
        super().step(increment)

        if not simulated_robot:
            print('ERROR, impossible to update goal')
            return
        try:
            if not self.is_finished(current_position=simulated_robot.position):
                # Command law
                target_position = simulated_robot.actions_to_update[
                    self.goal_id].target_position
                vector = utils.get_vector_3d(simulated_robot.position,
                                             target_position)
                dist = vector.get_norm()
                v_x = simulated_robot.max_speed['x']*(vector.x / dist)
                v_y = simulated_robot.max_speed['y']*(vector.y / dist)
                v_z = simulated_robot.max_speed['z']*(vector.z / dist)

                # We need to set the new speed with simulated robot lock
                with simulated_robot.lock:
                    simulated_robot.set_v_speed(v_x, v_y, v_z)
                    simulated_robot.energy -= (
                        increment
                        * simulated_robot.energy_cons.get("move", 0)
                        * simulated_robot.v_speed.get_norm())
                if simulated_robot.node is not None:
                    target_node = (
                        simulated_robot.nav_graph.convert_pos_into_node((
                            target_position.x, target_position.y)))
                    # Must be in increasing order to compare with
                    # forbiden_edges list.
                    simulated_robot.edge = (
                        min(simulated_robot.node, target_node),
                        max(simulated_robot.node, target_node))
                    simulated_robot.node = None
            else:
                with simulated_robot.lock:
                    simulated_robot.set_v_speed()
                    if simulated_robot.node is None:
                        simulated_robot.node = \
                            simulated_robot.nav_graph.convert_pos_into_node((
                                simulated_robot.position.x,
                                simulated_robot.position.y))

        except Exception:
            simulated_robot.simulation_node.get_logger().error(
                traceback.format_exc())

    def is_finished(self, current_position, eps=0.1):
        """
        This function checks if goal is finished.
        """
        if utils.euclidean_distance_2d(current_position,
                                       self.target_position) <= eps:
            return True
        else:
            return False

    def dist_to_finish(self, current_position):
        return euclidean_distance_2d(current_position, self.target_position)

    def fill_result(self, current_position):
        """
        This function fills a result message for the related GoToGoal.
        """
        return GoTo.Result(goal_id=self.goal_id,
                           reached_position=current_position,
                           total_travel_distance=self.current_travel_distance,
                           total_travel_time=self.current_duration)


class SimulatedRobot(BaseSimulatedRobot):
    """
    Mother class. Use AUV(SimulatedRobot) and surcharge functions by example.
    """

    def __init__(self, robot_id, simulation_node,
                 position=None, full_init=True):

        super().__init__(robot_id, simulation_node, full_init=full_init)

        self.robot_spec_data = self.simulation_node.robots_spec_data[
            self.robot_id]

        self.max_speed['x'] = self.robot_spec_data["robot_max_speed"]['x']
        self.max_speed['y'] = self.robot_spec_data["robot_max_speed"]['y']
        self.max_speed['z'] = self.robot_spec_data["robot_max_speed"]['z']

        self.robot_type = self.robot_spec_data["robot_type"]

        if position is not None:
            if len(position) >= 3:
                self.position = Point(
                    x=position[0], y=position[1], z=position[2])
            else:
                self.position = Point(
                    x=position[0], y=position[1], z=0.0)

        self.nav_graph = simulation_node.navigation_graph

        self.node = self.nav_graph.convert_pos_into_node(
            (self.position.x, self.position.y))

        self.edge = None

    def goto_execute_callback(self, goal_handle):
        """
        Function to execute a CAF GoTo goal.
        :param goal_handle: ROS2 Action GoalHandle object.
        :return: Result of the execution
        """

        # Check if goal is reached
        while not self.actions_to_update[
                goal_handle.request.goal_id].is_finished(self.position):
            if goal_handle.is_cancel_requested:
                self.simulation_node.get_logger().warn(
                    f"This goal handle must be canceled: {goal_handle}.")
                goal_handle.canceled()
                # renvoyer un result et retirer handler de actions to update.
                self.simulation_node.get_logger().warn(
                    f"This goal was canceled: {goal_handle}.")
                return Fibonacci.Result()
            # Wait for the next step call
            self.step_event.wait()

        # Goal is finished
        with self.lock:
            goto_hdl = self.actions_to_update.pop(goal_handle.request.goal_id)
            goal_handle.succeed()
            result = goto_hdl.fill_result(self.position)
            waypoint = \
                self.simulation_node.navigation_graph.convert_pos_into_node((
                    result.reached_position.x, result.reached_position.y), 1)
            self.simulation_node.last_visits[
                waypoint] = self.simulation_node.simulation_time
            # self.simulation_node.get_logger().info(
            #     'Returning result: {0}'.format(result)
            #     + '\nUpdated last visits: '
            #     + str(self.simulation_node.last_visits))
            return result

    def goto_goal_callback(self, goal_request, action_server_name=None):
        """
        Accept or reject a client request to begin an action.
        :param goal_request: received goal request
        :param action_server_name: action server name that received the goal
        :return: Goal response
        """
        with self.lock:
            self.simulation_node.get_logger().info(
            #     f"Received GoTo goal request from server {action_server_name}"
            #     f" with goal id : {goal_request.goal_id}"
                f"Received GoTo goal {goal_request.target_position}")
            # Create an AbstractActionHandler to increment it at each step
            self.simulation_node.publish_target_to_lrb(
                self.robot_id, goal_request)
            self.actions_to_update[goal_request.goal_id] = GoToHandler(
                goal_request)
            if self.robot_id in self.simulation_node.awaiting_goal_request:
                self.simulation_node.awaiting_goal_request.remove(
                    self.robot_id)
                # self.simulation_node.get_logger().info(
                #     f"Removed {self.robot_id} from awaiting list")

            if (self.simulation_node.awaiting_for_goto
                    and not self.simulation_node.awaiting_goal_request):
                self.simulation_node.simulation_timer.reset()
                # self.simulation_node.get_logger().info(
                #     "Reset simu timer from goto CB in simulated robots")
            return GoalResponse.ACCEPT

    def step(self, increment=1):
        """
        Step functions that update simulated robot states
        :param increment: time increment to use
        """
        with self.lock:
            # Update position
            if (self.energy > 0):
                # and self.simulation_node.moving_flags[self.robot_id]):
                self.update_goals(increment)
                self.update_position(increment)
                self.update_goals(increment)

                self.step_event.set()
                self.step_event.clear()
                # print('Step simulated ', self.robot_id)

                # Consume standby energy
                self.energy -= increment * self.energy_cons.get("standby", 0)

        # TODO put detection here ? (Even if here, not in caf ?)

    def abord_actions(self):
        for actions in self.actions_to_update:
            pass
