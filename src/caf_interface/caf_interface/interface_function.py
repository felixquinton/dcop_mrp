import rclpy
import json
import networkx as nx
from caf_interface.mission_display import MissionDisplay
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from caf_essential.auction import navigation_graph as nv
from caf_essential.base_node import BaseNode


class Interface(BaseNode):
    """
    ROS2 node receiving data from ground_truth topics. Transfers
    this data to caf_interface MissionDisplay for displaying.
    """

    def __init__(self):
        super().__init__('interface_node')

        # Stores the items to display
        self._initialize_interface()

        self.update_timer_period = 0.1
        self.update_timer = self.create_timer(self.update_timer_period,
                                              self.update_timer_cb)

    def _initialize_interface(self):
        # Stores the items to display
        self.items = {}

        self.initialize_topics()

        self.initialize_display()
        self.nav_graph = nv.navGraph(self.mission_spec_data)

    def update_timer_cb(self):
        """
        Updates the MissionDisplay object.
        """
        self.display.update_display()

    def initialize_display(self):
        """
        Initialize the MissionDisplay object with the navigation graph.
        """
        self.display = MissionDisplay()
        self.display.update_display()
        edges = {}
        for waypoint, waypoint_data in self.mission_spec_data.items():
            waypoint_id = waypoint.split('_')[-1]
            waypoint_pos = (waypoint_data["pos"][0],
                            self.display.canvas.winfo_height()
                            - waypoint_data["pos"][1])
            for neighbor in waypoint_data["neighbors"]:
                if not edges.get((neighbor, waypoint), False):
                    edges[(waypoint, neighbor)] = True
                    neighbor_pos = self.mission_spec_data[neighbor]["pos"]
                    neighbor_pos = (neighbor_pos[0],
                                    self.display.canvas.winfo_height()
                                    - neighbor_pos[1])
                    self.display.create_edge(waypoint_pos, neighbor_pos)
            self.display.create_waypoint(waypoint_pos, waypoint_id)

    def initialize_listeners(self):
        """
        Initialize the node's listeners. interface_node listens to the
        ground_truth topics from sim_robots for the robots' pose and energy
        status.
        """
        # Ground truth topics
        topic_ns = 'ground_truth'
        ns_list = []
        for robot_name in self.robots_list:
            ns_list.append([robot_name, topic_ns])

        # Pose
        topic_info = 'pose'
        msg_type = Pose
        listener_cb = self.update_pose_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1, ns_list=ns_list)
        # Energy
        topic_info = 'energy'
        msg_type = Float64
        listener_cb = self.update_ener_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1, ns_list=ns_list)

        # Communication Network
        topic_info = 'com_network'
        msg_type = String
        listener_cb = self.update_com_network_cb
        self.create_and_store_listeners(topic_info, msg_type,
                                        self.node_listeners,
                                        listener_cb=listener_cb,
                                        queue_size=1,
                                        ns_list=ns_list)

    def update_com_network_cb(self, msg, topic_name=None):
        com_graph = nx.readwrite.node_link_graph(json.loads(msg.data))
        self.display.update_com_network(com_graph)

    def update_pose_cb(self, msg, topic_name=None):
        """
        Callback called when receiving a pose message. Upon reception, calls
        methods from MissionDisplay to update robot's position on the display.
        :param msg: ROS2 msg of type Pose, the message stating updated pose
          data.
        :param topic_name:
        """
        robot_id = topic_name.split('/')[0]
        if robot_id not in self.items:
            self.items[robot_id] = self.display.create_robot(
                robot_id, self.robots_spec_data[robot_id]["com_range"])
        self.display.update_positions(self.items[robot_id],
                                      (msg.position.x,
                                       self.display.canvas.winfo_height()
                                       - msg.position.y))
        self.display.update_display()

    def update_ener_cb(self, msg, topic_name=None):
        """
        Callback called when receiving an energy message. Upon reception, calls
        methods from MissionDisplay to update robot's energy on the display.
        :param msg: ROS2 msg of type Float64, the message stating updated
          energy data.
        :param topic_name:
        """
        robot_id = topic_name.split('/')[0]
        self.display.update_energy(robot_id, self.items[robot_id], msg.data)
        self.display.update_display()


def main(args=None):
    rclpy.init(args=args)

    interface_node = Interface()

    rclpy.spin(interface_node)

    interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
