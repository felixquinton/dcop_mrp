from cs_tools import SurfaceType, GeoPoint, GeoProj, CoordinateType
import json


def read_from_must_hmi(filename=""):
    """Parse a path.json file from must_hmi and convert it into a nx graph.
    :param filename: str, the path to the must_hmi json file.
    """
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
        raise
    edges = data["links"]
    nodes = data["nodes"]
    for node in nodes:
        point = GeoPoint(node["latitude"], node["longitude"], node["altitude"])
        cart_pos = geo_proj.spherical_to_local(point, coord_type)
        nx_graph_dic[f"waypoint_{node['id']+1}"] = {
            "pos": [cart_pos.x, cart_pos.y],
            "neighbors": []
        }
    for edge in edges:
        nx_graph_dic[f"waypoint_{edge['source']+1}"]["neighbors"].append(
            f"waypoint_{edge['target']+1}")
    return nx_graph_dic


if __name__ == "__main__":
    print(read_from_must_hmi("path.json"))
