import numpy as np
import networkx as nx


class navGraph(nx.Graph):
    """
    Class to handle the navigation graph describing the scenario's spacial
    environment.
    """

    def __init__(self, mission_spec_data=None):
        """
        Initialize the graph with the mission data. Note that as the navigation
        graph is initialized from a node's mission specs, different nodes
        are able to consider different navigation graph.
        :param mission_spec_data: dict, resulting from
          utils.get_mission_spec_data
        """
        super().__init__()
        self._build_pos_dic(mission_spec_data)
        self._build_edges(mission_spec_data)

    def _build_pos_dic(self, mission_spec_data):
        """
        Initialize the graph's nodes.
        :param mission_spec_data: dict, resulting from
          utils.get_mission_spec_data
        """
        for waypoint, waypoint_data in mission_spec_data.items():
            waypoint_id = waypoint.split('_')[-1]
            self.add_node(int(waypoint_id), pos=waypoint_data["pos"])

    def _build_edges(self, mission_spec_data):
        """
        Initialize the graph's edges.
        :param mission_spec_data: dict, resulting from
          utils.get_mission_spec_data
        """
        positions = nx.get_node_attributes(self, "pos")
        for waypoint, waypoint_data in mission_spec_data.items():
            waypoint_id = int(waypoint.split('_')[-1])
            waypoint_pos = np.array(positions[waypoint_id])
            for neighbor in waypoint_data["neighbors"]:
                neighbor_id = int(neighbor.split('_')[-1])
                neighbor_pos = np.array(positions[neighbor_id])
                if not (neighbor_id, waypoint_id) in self.edges:
                    dist = np.linalg.norm(waypoint_pos-neighbor_pos)
                    self.add_weighted_edges_from(
                        [(waypoint_id, neighbor_id, dist)])
            # Create loops so that robot doesn't need to move to go to
            # the waypoint they are at.
            self.add_weighted_edges_from(
                [(waypoint_id, waypoint_id, 0.)])

    def shortest_path(self, w1, w2):
        """
        Compute the shortest path from node w1 to node w2.
        :param w1: int, the node of the graph which is the start of the path.
        :param w2: int, the node of the graph which is the end of the path.
        :return: list of int, the ordered list of the nodes that compose the
          shortest path.
        """
        return(nx.dijkstra_path(self, source=w1, target=w2, weight='weight'))

    def path_length(self, path, weight='weight'):
        """
        Compute the length of a path as the sum of its edges.
        :param path: list of int, the path to be evaluated.
        :param weight: string, the tag of the attribute corresponding to the
          edges' weights in the graph object.
        :return: float, the length of the path.
        """
        return sum([self.edges[(path[i], path[i+1])][weight]
                    for i in range(len(path)-1)])

    def shortest_path_length(self, w1, w2):
        """
        Use shortest_path and path_length to compute the length of the shortest
        path between two nodes.
        :param w1: int, the node of the graph which is the start of the path.
        :param w2: int, the node of the graph which is the end of the path.
        :return: float, the length of the path shortest path for w1 to w2.
        """
        return(self.path_length(self.shortest_path(w1, w2)))

    def convert_pos_into_node(self, pos, eps=0.1):
        """
        Converts a 2D pos into a node id.
        :param pos: 2D vector, the position of the node to retrieve.
        :return: int, the id of the node corresponding to the 2D position.
        """
        try:
            p1, p2 = pos
        except Exception:
            raise ValueError("pos argument was not 2D.")
        array_given_pos = np.array(pos)
        for node_id, node_pos in zip(
                self.nodes, nx.get_node_attributes(self, "pos").values()):
            array_node_pos = np.array(node_pos)
            if np.linalg.norm(array_given_pos-array_node_pos) < eps:
                return node_id

    def convert_node_into_pos(self, node_id):
        """
        Converts a node id into a 2D pos.
        :param node_id: int, the id of the node to retrieve.
        :return: 2D vector, the 2D position of the node corresponding to the id
        """
        if node_id in self.nodes:
            return nx.get_node_attributes(self, "pos")[node_id]
        else:
            raise KeyError("Node id %d not in graph.", node_id)
            return None


if __name__ == "__main__":
    G = navGraph()
