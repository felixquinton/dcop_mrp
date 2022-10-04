import numpy as np
import networkx as nx
import csv
import json


class comGraph(nx.DiGraph):
    """
    Class to handle the navigation graph describing the scenario's spacial
    environment.
    """

    def __init__(self, robots_spec_data=None, proba_matrix=[]):
        """
        Initialize the graph with the mission data. Note that as the navigation
        graph is initialized from a node's mission specs, different nodes
        are able to consider different navigation graph.
        :param robots_spec_data: dict, resulting from
          utils.get_robots_spec_data
        :param obstacle_edges: list of pair of edges for which coms are blocked
          by an obstacle.
        """
        super().__init__()
        self.robots_ranges = {}
        self._build_nodes(robots_spec_data)
        self.proba_matrix = self._create_proba_matrix(proba_matrix)

    def _create_proba_matrix(self, mat):
        if not mat:
            # A compléter pour faire un tableau de 1.
            return 1
        else:
            return mat

    def _build_nodes(self, robots_spec_data):
        """
        Initialize the graph's nodes.
        :param robots_spec_data: dict, resulting from
          utils.get_robots_spec_data
        """
        self.nb_robots = 0
        for robot, robot_data in robots_spec_data.items():
            self.nb_robots += 1
            robot_id = robot.split('_')[-1]
            self.add_node(int(robot_id))
            self.robots_ranges[robot] = robot_data["com_range"]

    def update_edges(self, simulated_robots, logging=True):
        """
        Initialize the graph's edges.
        :param simulated_robots: dict, simulated robots from the sim.
        :return modified: boolean, True iif the CN changed
        """
        modified = False
        prior_edges = list(self.edges)
        self.remove_edges_from(prior_edges)
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
                            [(robot_id, teammate_id,
                              self.loss_proba(robot, teammate))])
                if (np.linalg.norm(robot_pos-teammate_pos)
                        <= self.robots_ranges[teammate.robot_id]):
                    self.add_weighted_edges_from(
                            [(teammate_id, robot_id,
                              self.loss_proba(robot, teammate))])
        if (logging
                and (not all([e in list(self.edges) for e in prior_edges])
                     or not all([e in prior_edges for e in list(self.edges)]))):
            modified = True
        return modified

    def convert_to_str(self):
        return json.dumps(nx.readwrite.json_graph.node_link_data(self))

    def loss_proba(self, r1, r2, dist=0):
        """
        Computes the probability of losing coms.
        # TODO: Use distance to compute non-binary probability
        """

        if r1.robot_id == r2.robot_id:
            return 1
        if r1.edge is not None and r2.edge is not None:
            if r1.edge == r2.edge:
                return 1
            elif str((r1.edge, r2.edge)) in self.proba_matrix:
                return self.proba_matrix[str((r1.edge, r2.edge))]
            elif str((r2.edge, r1.edge)) in self.proba_matrix:
                return self.proba_matrix[str((r2.edge, r1.edge))]
            # This last return means that one of the robots is on a node.
            # This happend just at the beginning, so always com...
        elif r1.edge is None and r1.node is not None:
            if r2.edge is not None:
                if str((r1.node, r2.edge)) in self.proba_matrix:
                    return self.proba_matrix[str((r1.node, r2.edge))]
            elif r2.node is not None:
                if str((r1.node, r2.node)) in self.proba_matrix:
                    return self.proba_matrix[str((r1.node, r2.node))]
        elif r2.edge is None and r2.node is not None:
            if r1.edge is not None:
                if str((r2.node, r1.edge)) in self.proba_matrix:
                    return self.proba_matrix[str((r2.node, r1.edge))]
            elif r1.node is not None:
                if str((r2.node, r1.node)) in self.proba_matrix:
                    return self.proba_matrix[str((r2.node, r1.node))]
        return 1

    def remove_integer_nodes(self, nb_robots):
        """This method should actually not exist, as these nodes shouldnt be
        created in the first place. But it is useful as a quick and dirty
        alternative.
        :param nb_robots: int, number of robots.
        """
        self.remove_nodes_from([r for r in range(1, nb_robots+1)])

    def get_data(self):
        """Returns the data to be logged.
        """
        self.remove_integer_nodes(self.nb_robots)
        H = self.to_undirected()
        cs = nx.connected_components(H)
        S = {str(c): H.subgraph(c).copy() for c in cs}
        return {str(c): {'size': S[str(c)].number_of_nodes(),
                         'nb_edges': (S[str(c)].number_of_edges()
                                      - S[str(c)].number_of_nodes())}
                for c in S}

    def log(self, time, path):
        data = json.dumps(self.get_data())
        with open(path, mode='a+') as outfile:
            csv_writer = csv.writer(outfile, delimiter=',',
                                    quotechar='"',
                                    quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow([
                time, data,
                self.convert_to_str()])


if __name__ == "__main__":
    G = comGraph()
