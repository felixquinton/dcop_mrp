from rclpy.action import ActionServer, CancelResponse, GoalResponse

# TODO find how to use UUID in ROS2 msg rather than string
# from unique_identifier_msgs.msg import UUID

from geometry_msgs.msg import Point

# TODO rebuild cb organisation and delete this line (GoTo for cb result)
from caf_messages.action import GoTo, AbstractAction

from caf_essential.base_node import ServerHandler
from caf_essential.utils import utils, get_actions

from functools import partial
import threading


class ActionHandler:
    """
    This class define Handler for received Action Goal
    """

    def __init__(self, goal_request):
        # Basic information
        self.goal_id = goal_request.goal_id
        self.goal_request = goal_request
        # Goal execution duration
        self.current_duration = 0.0

    def step(self, increment, *args, **kwargs):
        """
        Step function to increment execution duration.
        Can be surcharged for specific Action behaviour
        :param increment: time increment to use
        :param args: others args to allow surcharge
        :param kwargs: others args to allow surcharge
        """
        self.current_duration += increment

    def is_finished(self, *args, **kwargs):
        """
        This function checks if goal is finished.
        NEEDS to be surcharged.
        """
        pass

    def fill_result(self, *args, **kwargs):
        """
        This function fills a result message for the related ActionGoal.
        NEEDS to be surcharged.
        """
        pass


class AbstractActionHandler(ActionHandler):
    """
    Class to handle CAF AbstractAction goal.
    """

    def __init__(self, goal_request):
        super(AbstractActionHandler, self).__init__(goal_request)

        self.target_duration = goal_request.target_duration

    def step(self, increment, *args, **kwargs):
        """
        Step function to increment execution duration.
        :param increment: time increment to use
        """
        super().step(increment)

    def is_finished(self, *args, **kwargs):
        """
        This function checks if goal is finished.
        """
        if self.current_duration > self.target_duration:
            return True
        else:
            return False

    def fill_result(self):
        """
        This function fills a result message for the related
        AbstractActionGoal.
        """
        return AbstractAction.Result(goal_id=self.goal_id,
                                     total_duration=self.current_duration)


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
        :param simulated_robot: a simulated robot object -
         robot executing the goal
        """
        super().step(increment)

        if not simulated_robot:
            print('ERROR, impossible to update goal')
            return

        if not self.is_finished(current_position=simulated_robot.position):
            # Command law
            lim_dist = 50.0
            target_position = simulated_robot.actions_to_update[
                self.goal_id].target_position
            vector = utils.get_vector_3d(
                simulated_robot.position, target_position)
            dist = vector.get_norm()
            if dist >= lim_dist:
                # use maximum velocity
                c = 1.0
            else:
                # Reduce velocity when near to goal=
                c = dist / lim_dist
            v_x = simulated_robot.max_speed['x']*(vector.x / dist)*c
            v_y = simulated_robot.max_speed['y']*(vector.y / dist)*c
            v_z = simulated_robot.max_speed['z']*(vector.z / dist)*c

            # We need to set the new speed with simulated robot lock
            with simulated_robot.lock:
                simulated_robot.set_v_speed(v_x, v_y, v_z)
                simulated_robot.energy -= (
                    increment
                    * simulated_robot.energy_cons.get("move", 0)
                    * simulated_robot.v_speed.get_norm())
        else:
            with simulated_robot.lock:
                simulated_robot.set_v_speed()

    def is_finished(self, current_position, eps=0.1):
        """
        This function checks if goal is finished.
        """
        if utils.euclidean_distance_2d(
                current_position, self.target_position) <= eps:
            return True
        else:
            return False

    def fill_result(self, current_position):
        """
        This function fills a result message for the related GoToGoal.
        """
        return GoTo.Result(goal_id=self.goal_id,
                           reached_position=current_position,
                           total_travel_distance=self.current_travel_distance,
                           total_travel_time=self.current_duration)


class SimulatedRobot:
    """
    Mother class. Use AUV(SimulatedRobot) and surcharge functions by example.
    """

    def __init__(self, robot_id, simulation_node, full_init=True):
        # Simulated robot id
        self.robot_id = robot_id
        # The node attribute allow to access node information and logger
        self.simulation_node = simulation_node

        # Mutex dedicated to this SimulatedRobot, useful to manage data access
        # from callbacks
        if full_init:
            self.lock = threading.RLock()

            # Allow to pause callback until step is run again to avoid infinite
            # calculation
            self.step_event = threading.Event()

        # Init robot data
        self.robot_spec_data = self.simulation_node.robots_spec_data[
            self.robot_id]
        try:
            self.fl_actions_package_name = self.robot_spec_data[
                'fl_actions_package_name']
        except KeyError:
            self.fl_actions_package_name = "caf_messages"

        # speed (m/s)
        self.v_speed = utils.Vector3D()
        self.norm_speed = 0.0

        # Check start parameters
        self.position = Point()
        self.energy = 0.0
        self.energy_cons = self.robot_spec_data["robot_energy_cons"]
        self.max_speed = {}
        self.init_robot_data()

        # Robot actions simulation
        if full_init:
            self.robot_actions_servers = {}
            self.initialize_actions_servers()

        # TODO manage this in a more proper way ? Without a dedicated dict
        self.actions_to_update = {}

    def init_robot_data(self):
        """
        This function init all robot information with specified start
        parameters such as energy or spawn position
        """
        # Spawn position
        self.init_spawn_position()
        # Energy (float)
        self.init_energy()
        # max_speed
        # TODO use param
        self.max_speed['x'] = 20.0
        self.max_speed['y'] = 20.0
        self.max_speed['z'] = 3.0

    def set_v_speed(self, x=0.0, y=0.0, z=0.0):
        """
        Sets a speed for the robot.
        :param x: Speed on x axis
        :param y: Speed on y axis
        :param z: Speed on z axis
        """
        self.v_speed.set_vector(x, y, z)
        self.norm_speed = self.v_speed.get_norm()

    def init_spawn_position(self):
        """
        This function initialize robot position with the spawn position
        indicated in robots spec data.
        """
        try:
            self.position = Point(x=self.robot_spec_data["robot_spawn_pos"][0],
                                  y=self.robot_spec_data["robot_spawn_pos"][1])
        except KeyError:
            self.simulation_node.get_logger().warn(
                'No specified spawn position for robot ' + self.robot_id)
            return

    def init_energy(self):
        """
        This function initialize robot energy with the spawn energy indicated
        in robots spec data.
        """
        try:
            self.energy = self.robot_spec_data["robot_energy"]

        except KeyError:
            self.simulation_node.get_logger().warn(
                'No specified energy for robot ' + self.robot_id)
            return

    def get_action_server_cb(self, action_type):
        """
        Return a dict with the callback to use to init this type of
        ActionServer.
        Surcharge this function to adapt CAF to your use case
        :param action_type: Action type of the ActionServer
        :return: dict of callbacks to use
        """
        # Use CAF default CB
        if action_type == GoTo:
            cb_dict = {'execute_callback': self.goto_execute_callback,
                       'callback_group': self.simulation_node.callback_group,
                       'goal_callback': self.goto_goal_callback,
                       'cancel_callback': self.default_cancel_callback}
        elif action_type == AbstractAction:
            cb_dict = {
                'execute_callback': self.abstract_action_execute_callback,
                'callback_group': self.simulation_node.callback_group,
                'goal_callback': self.abstract_action_goal_callback,
                'cancel_callback': self.default_cancel_callback}
        else:
            self.simulation_node.get_logger().error(
                'action type does not match callbacks specification')
            return
        return cb_dict

    def initialize_actions_servers(self):
        """
        Initialize all actions servers of the simulated robot
        """
        # ------------- Robots actions requests
        servers_ns = 'fl_actions'
        # Initialize dicts
        self.robot_actions_servers[servers_ns] = {}

        # Create actions servers
        robot_actions = self.robot_spec_data['actions']
        for action_name in robot_actions:
            action_type = get_actions.get_action_type(
                action_name, self.fl_actions_package_name)
            if not action_type:
                continue
                # TODO replace by exception
            # print('initialize server', self.robot_id,':', action_name)
            action_server_name = utils.str_builder([self.robot_id, servers_ns,
                                                    action_name], '/')
            cb_dict = self.get_action_server_cb(action_type)

            # Did not find a callback specification for this action type
            if not cb_dict:
                continue

            # handle_accepted_callback = self.handle_accepted_callback
            server = ActionServer(self.simulation_node,
                                  action_type,
                                  action_server_name,
                                  execute_callback=cb_dict['execute_callback'],
                                  # TODO manage handle_accepted callback ?
                                  # handle_accepted_callback=handle_accepted_callback,
                                  callback_group=cb_dict['callback_group'],
                                  goal_callback=partial(
                                      cb_dict['goal_callback'],
                                      action_server_name=action_server_name),
                                  cancel_callback=cb_dict['cancel_callback'])
            server_hdl = ServerHandler(action_name,
                                       action_type,
                                       action_server_name,
                                       server,
                                       cb_dict)
            self.robot_actions_servers[servers_ns][action_name] = server_hdl

    def default_goal_callback(self, goal_request, action_server_name=None):
        """Accept or reject a client request to begin an action."""
        with self.lock:
            self.simulation_node.get_logger().info('Received goal request')
            return GoalResponse.ACCEPT

    def default_cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        with self.lock:
            self.simulation_node.get_logger().info(
                f'Received cancel request {goal_handle}')
            return CancelResponse.ACCEPT

    def default_execute_callback(self, goal_handle):
        """
        Default function to execute a goal.
        :param goal_handle: ROS2 Action GoalHandle object.
        :return: Result of the execution
        """
        with self.lock:
            self.simulation_node.get_logger().info('Executing goal...')
            goal_handle.succeed()
            result = None
            self.simulation_node.get_logger().info(
                'Returning result: {0}'.format(result))
            return result

    def default_handle_accepted_callback(self, goal_handle):
        pass

    # ---------------- GoTo callbacks -----------------------------------------
    def goto_goal_callback(self, goal_request, action_server_name=None):
        """
        Accept or reject a client request to begin an action.
        :param goal_request: received goal request
        :param action_server_name: action server name that received the goal
        :return: Goal response
        """
        with self.lock:
            self.simulation_node.get_logger().info('Received GoTo goal request'
                                                   + ' from server :'
                                                   + action_server_name
                                                   + ' with goal id : '
                                                   + goal_request.goal_id
                                                   )
            # Create an AbstractActionHandler to increment it at each step
            self.actions_to_update[goal_request.goal_id] = GoToHandler(
                goal_request)
            return GoalResponse.ACCEPT

    def goto_execute_callback(self, goal_handle):
        """
        Function to execute a CAF GoTo goal.
        :param goal_handle: ROS2 Action GoalHandle object.
        :return: Result of the execution
        """
        with self.lock:
            self.simulation_node.get_logger().info(
                self.robot_id + ' Executing goal: GoTo '
                + str(self.actions_to_update[
                    goal_handle.request.goal_id].target_position))

        # Check if goal is reached
        while not self.actions_to_update[
                goal_handle.request.goal_id].is_finished(self.position):
            # Wait for the next step call
            self.step_event.wait()

        # Goal is finished
        with self.lock:
            goto_hdl = self.actions_to_update.pop(goal_handle.request.goal_id)
            goal_handle.succeed()
            result = goto_hdl.fill_result(self.position)
            self.simulation_node.get_logger().info(
                'Returning result: {0}'.format(result))
            return result

    # ---------------- AbstractAction callbacks -------------------------------
    def abstract_action_goal_callback(self, goal_request,
                                      action_server_name=None):
        """Accept or reject a client request to begin an action."""
        with self.lock:
            self.simulation_node.get_logger().info(
                "Received AbstractAction goal request from server : \n"
                f"{action_server_name} \n"
                f"with goal id : {goal_request.goal_id}")
            # Create an AbstractActionHandler to increment it at each step
            self.actions_to_update[
                goal_request.goal_id] = AbstractActionHandler(goal_request)
            return GoalResponse.ACCEPT

    def abstract_action_execute_callback(self, goal_handle):
        """
        Function to execute a CAF AbstractAction goal.
        :param goal_handle: ROS2 Action GoalHandle object.
        :return: Result of the execution
        """
        with self.lock:
            self.simulation_node.get_logger().info('Executing goal...')

        # Check if goal is finished at each new call of step
        while not self.actions_to_update[
                goal_handle.request.goal_id].is_finished():
            self.step_event.wait()

        # Goal is finished
        with self.lock:
            abstract_action_hdl = self.actions_to_update.pop(
                goal_handle.request.goal_id)
            goal_handle.succeed()
            result = abstract_action_hdl.fill_result()
            self.simulation_node.get_logger().info(
                'Returning result: {0}'.format(result))
            return result

    # --------------- Steps functions to run simulation
    def update_position(self, time_increment):
        """
        Update robot position according to its current speed
        :param time_increment: time increment to use
        """
        with self.lock:
            new_x = self.position.x + self.v_speed.x*time_increment
            new_y = self.position.y + self.v_speed.y*time_increment
            new_z = self.position.z + self.v_speed.z*time_increment
            self.position.x = new_x
            self.position.y = new_y
            self.position.z = new_z

    def update_goals(self, increment):
        """
        Calls ActionHandlers step function
        :param increment: time increment to use
        """
        with self.lock:
            for action_hdl in self.actions_to_update.values():
                action_hdl.step(increment, simulated_robot=self)

    def step(self, increment=1):
        """
        Step functions that update simulated robot states
        :param increment: time increment to use
        """
        with self.lock:
            # Update position
            if (self.energy > 0):
                # and self.simulation_node.moving_flags[self.robot_id]):
                self.update_position(increment)

                self.update_goals(increment)
                self.step_event.set()
                self.step_event.clear()
                # print('Step simulated ', self.robot_id)

                # Consume standby energy
                self.energy -= increment * self.energy_cons.get("standby", 0)

        # TODO put detection here ? (Even if here, not in caf ?)
