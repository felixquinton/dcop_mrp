import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor

# TODO find how to use UUID in ROS2 msg rather than string
# from unique_identifier_msgs.msg import UUID

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64, String
import numpy as np

from caf_essential.base_node import BaseNode
from caf_sim_robots.simulated_robot import SimulatedRobot
from caf_essential.utils import com_network as cn


class SimRobots(BaseNode):
    """
    This is the simulation node for CAF. This node do several things:
    - publish ground truth
    - use simulated robots classes to manage robots information
    - call a step function with a given time increment to update system states
    This node rely on SimulatedRobot class to call their corresponding step
    function at each run of the simulation.
    """

    def __init__(self, node_name='simulation_robots_node',
                 init_sim_robots="True", init_simulation_timer=True):
        super().__init__(node_name)

        self.initialize_topics()

        # Define a cb group to run simulation concurrently with simulated robots cb
        self.callback_group = ReentrantCallbackGroup()

        # ------------------- init Simulated robot -------------------------
        # Store information about robots
        if init_sim_robots:
            self.simulated_robots = {}
            self.initialize_simulated_robots()

        # ---------- INIT SIMULATION ATTRIBUTES -------------------------
        # init simulation_time
        self.simulation_time = 0

        # Use this variables to speed up or slow down the simulation
        # TODO use a parameter in config or launch file
        self.time_factor = 1

        # Init com network
        self.com_network = cn.comGraph(self.robots_spec_data)

        # Simulation timer, this timer must be sufficient to do all calculation
        self.simulation_timer_period = .075
        if init_simulation_timer:
            self.simulation_timer = self.create_timer(
                self.simulation_timer_period, self.run_simulation,
                callback_group=self.callback_group)

    def update_random_position_timer_cb(self):
        """
        This function is useful to use random position
        """
        for robot_id in self.robots_list:
            # Generate random coord
            pos = self.generate_random_coord()
            new_position = Point(x=pos[0], y=pos[1])
            self.simulated_robots[robot_id].position = new_position

    def generate_random_coord(self):
        x = np.random.uniform(0, 600)
        y = np.random.uniform(0, 400)
        return x, y

    def initialize_publishers(self):
        """
        Initialize all publishers of the node
        """
        # -------------- Ground truth
        self.initialize_ground_truth_publishers()

        # --------------
        # others publishers

    def initialize_ground_truth_publishers(self):
        """"Initialize ground truth publishers for each robot
        """
        topic_ns = 'ground_truth'
        ns_list = []
        for robot_id in self.robots_list:
            ns_list.append([robot_id, topic_ns])

        # Pose
        topic_info = 'pose'
        msg_type = Pose
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers, queue_size=100,
                                         ns_list=ns_list)

        # Energy
        topic_info = 'energy'
        msg_type = Float64
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers, queue_size=100,
                                         ns_list=ns_list)
        # Communication Network
        topic_info = 'com_network'
        msg_type = String
        self.create_and_store_publishers(topic_info, msg_type,
                                         self.node_publishers, queue_size=100,
                                         ns_list=ns_list)
        # ...
        # TODO add robot state (HS/ON), nav state and sensors states

    def initialize_simulated_robots(self):
        """
        This function creates class to handle CAF simulated robots
        NEEDS to be surcharged with your use case (and so you SimulatedRobot classes)
        """
        for robot_id in self.robots_list:
            self.simulated_robots[robot_id] = SimulatedRobot(robot_id, self)

    def publish_ground_truth(self):
        """
        This function publish ground truth information for each simulated robots
        """
        topic_ns = 'ground_truth'
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

            # Communication Network
            topic_info = 'com_network'
            topic_keys = [robot_id, topic_ns, topic_info]
            msg = String(data=self.com_network.convert_to_str())
            self.publish_msg(msg, topic_keys)

    def run_simulation(self):
        """
        This function runs the simulation.
        Increment is the "time increment" resulting from the time factor and timer step.
        This function is called periodically by a timer.
        """
        with self.lock:
            increment = self.time_factor*self.simulation_timer_period
            self.simulation_time += increment

            for robot in self.simulated_robots.values():
                robot.step(increment)

            self.com_network.update_edges(self.simulated_robots)

            self.publish_ground_truth()

        # print('run simulation, simulation time is '+str(self.simulation_time))


def main(args=None):
    rclpy.init(args=args)

    node = SimRobots()

    executor = MultiThreadedExecutor(num_threads=20)
    rclpy.spin(node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
