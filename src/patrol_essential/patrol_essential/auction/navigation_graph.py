from caf_essential.auction import navigation_graph as nv
# from cs_tools import SurfaceType, GeoPoint, GeoProj, CoordinateType
import json


class navGraph(nv.navGraph):
    """
    Class specific to patrolling scenarios to handle the navigation graph
    describing the scenario's spacial environment.
    """

    def __init__(self, graph_as_dict=None, must_hmi_filename=None):
        """
        Initialize the graph with the mission data. Note that as the navigation
        graph is initialized from a node's mission specs, different nodes
        are able to consider different navigation graph.
        :param graph_as_dict: dict, representing the graph from
            get_mission_spec_data
        :param must_hmi_filename: str, path to a json from must_hmi
        """
        if must_hmi_filename is not None:
            graph_as_dict = self.read_from_must_hmi(must_hmi_filename)

        super().__init__(graph_as_dict)

        self._init_nodes_sensors()

    def _build_pos_dic(self, graph_as_dict):
        """
        Initialize the graph's nodes.
        :param graph_as_dict: dict, representing the graph from
            get_mission_spec_data
        """
        for waypoint, waypoint_data in graph_as_dict.items():
            waypoint_id = waypoint.split('_')[-1]
            self.add_node(int(waypoint_id),
                          pos=waypoint_data["pos"],
                          idleness=0)

    def edit_nodes_sensors(self, sensors_dic):
        """
        Create or edit the "sensors" attribute that indicate which sensor is
        needed to survey each node.
        :param sensors_dic: dict, linking nodes to a list of sensors strings
        Note: sensors_dic may include any number of the graph's nodes.
        """
        try:
            for node, sensors in sensors_dic.items():
                self.nodes[node]["sensors"] = sensors
        except KeyError as e:
            print(f"Error {e} ; Nodes of the graph: {self.nodes(data=True)}")
            raise

    def _init_nodes_sensors(self):
        """
        Init the "sensors" attribute that indicate which sensor is needed to
        survey each node, with each node being available for any sensor.
        :param sensors_dic: dict, linking nodes to a list of sensors strings
        """
        for node in self.nodes:
            self.nodes[node]["sensors"] = ["day_camera",
                                           "night_camera",
                                           "lidar"]

    """
    def read_from_must_hmi(self, filename=""):
        nx_graph_dic = {}
        surf = SurfaceType(1)
        origin = GeoPoint(44.272792, 1.726613, -1)
        geo_proj = GeoProj(surf, origin)
        coord_type = CoordinateType(1)
        try:
            with open(filename) as json_file:
                data = json.load(json_file)
        except FileNotFoundError:
            raise
        except Exception as e:
            raise e
        edges = data["links"]
        nodes = data["nodes"]
        for node in nodes:
            point = GeoPoint(
                node["latitude"], node["longitude"], node["altitude"])
            cart_pos = geo_proj.spherical_to_local(point, coord_type)
            nx_graph_dic[f"waypoint_{node['id']+1}"] = {
                "pos": [cart_pos.x, cart_pos.y],
                "neighbors": []
            }
        for edge in edges:
            nx_graph_dic[f"waypoint_{edge['source']+1}"]["neighbors"].append(
                f"waypoint_{edge['target']+1}")
        return nx_graph_dic
    """


if __name__ == "__main__":
    G = navGraph()
