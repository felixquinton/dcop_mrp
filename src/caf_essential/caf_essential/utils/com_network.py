import numpy as np
import networkx as nx
import json


class comGraph(nx.DiGraph):
    """
    Class to handle the navigation graph describing the scenario's spacial
    environment.
    """

    def __init__(self, robots_spec_data=None):
        """
        Initialize the graph with the mission data. Note that as the navigation
        graph is initialized from a node's mission specs, different nodes
        are able to consider different navigation graph.
        :param mission_spec_data: dict, resulting from
          utils.get_mission_spec_data
        """
        super().__init__()
        self.robots_ranges = {}
        self._build_nodes(robots_spec_data)

    def _build_nodes(self, robots_spec_data):
        """
        Initialize the graph's nodes.
        :param robots_spec_data: dict, resulting from
          utils.get_robots_spec_data
        """
        for robot, robot_data in robots_spec_data.items():
            robot_id = robot.split('_')[-1]
            self.add_node(int(robot_id))
            self.robots_ranges[robot] = robot_data["com_range"]

    def update_edges(self, simulated_robots):
        """
        Initialize the graph's edges.
        :param simulated_robots: dict, simulated robots from the sim.
        """
        self.remove_edges_from(list(self.edges))
        for robot in simulated_robots.values():
            robot_pos = np.array((robot.position.x, robot.position.y))
            robot_id = robot.robot_id.split('_')[-1]
            for teammate in simulated_robots.values():
                teammate_pos = np.array((teammate.position.x,
                                         teammate.position.y))
                teammate_id = teammate.robot_id.split('_')[-1]
                if (np.linalg.norm(robot_pos-teammate_pos)
                        <= self.robots_ranges[robot.robot_id]):
                    self.add_weighted_edges_from(
                        [(robot_id, teammate_id, self.loss_proba())])
                if (np.linalg.norm(robot_pos-teammate_pos)
                        <= self.robots_ranges[teammate.robot_id]):
                    self.add_weighted_edges_from(
                        [(teammate_id, robot_id, self.loss_proba())])

    def convert_to_str(self):
        return json.dumps(nx.readwrite.json_graph.node_link_data(self))

    def loss_proba(self, dist=0):
        """
        Computes the probability of losing coms.
        # TODO: Use distance to compute non-binary probability
        """
        return 1


if __name__ == "__main__":
    G = comGraph()
