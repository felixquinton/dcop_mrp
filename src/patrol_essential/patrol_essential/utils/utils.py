import os
import json
import csv

from ament_index_python.packages import get_package_share_directory

# Messages for patrol auction processes
from caf_messages.msg import Award
from patrol_messages.msg import Header

from builtin_interfaces.msg import Time
from math import sqrt, pow


def get_json_data(file_path):
    """
    Loads and returns data from a json file
    """
    with open(file_path, 'r') as f:
        return json.load(f)


def str_builder(words, sym='/'):
    """
    Builds a string chain from a list of words using a separator
    :param words: words to concatenate
    :param sym: symbol to use as a separator
    :return: string chain
    """
    res = ''
    for i in range(len(words)):
        word = words[i]
        if not word:
            continue
        res += word
        if i != len(words) - 1:
            res += sym
    return res


def import_generated_nav_graph(data):
    graph_dic = {f'waypoint_{n["id"]}': {
        "pos": n['pos'],
        "neighbors": [
            f'waypoint_{e["target"]}'
            for e in data["graph"]["links"] if e["source"] == n["id"]]
        + [f'waypoint_{e["source"]}'
           for e in data["graph"]["links"] if e["target"] == n["id"]
           ]}
                 for n in data['graph']['nodes']}
    return graph_dic


def get_dict_value_list_keys(d, keys, init=False,
                             return_element='last_value'):
    """
    Return value of a dict given a list of keys
    :param d: dict
    :param keys: list of keys
    :param init: Boolean. If a key is missing will init value with
    another dict
    :param return_element: last element to return, can be the last
    value or the dict just before
    :return: d value for list of keys
    """
    second_to_last = None
    last_value = d
    for key in keys:
        try:
            second_to_last = last_value
            last_value = last_value[key]
        except KeyError:
            if init:
                last_value[key] = {}
                second_to_last = last_value
                last_value = last_value[key]
            else:
                raise
    if return_element == 'second_to_last':
        return second_to_last
    elif return_element == 'last_value':
        return last_value
    else:
        print('ERROR, get_dict_value_list_keys')


def replace_character_from_list(li, character, new_character=''):
    """
    This function replaces a character in a list.
    Useful by example to manages specific topic names.
    :param li: list to use
    :param character: character to replace
    :param new_character: new character to use
    :return: modified list
    """
    new_list = []
    for elem in li:
        if isinstance(elem, str):
            elem = elem.replace(character, new_character)
            new_list += [elem]
        elif isinstance(elem, list):
            replace_character_from_list(elem, character)
        else:
            print('ERROR: replace_character_from_list not a valid type')
    return new_list


def get_team_members_data(node):
    """
    Get team members parameter to find the spec file to use.
    Then load and return the data from the team member json file
    :param node: node to use to retrieve data
    :return: dict with team members data
    """
    team_members_file_name = node.get_parameter('team_members_file_name')
    if team_members_file_name.value == 'default_team_spec':
        node.team_members_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', team_members_file_name.value)
    else:
        node.team_members_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', team_members_file_name.value)
    node.get_logger().info('Team specification file used: '
                           + team_members_file_name.value)
    dict = get_json_data(node.team_members_file_path)
    res = {"team_members_list": []}
    for robot_key in dict:
        res["team_members_list"].append(dict[robot_key]["id"])
    return res


def get_robots_spec_data(node):
    """
    Get team members parameter to find the spec file to use.
    Then load and return the data from the team member json file
    :param node: node to use to retrieve data
    :return: dict with robots spec data
    """
    robots_spec_file_name = node.get_parameter('robots_spec_file_name')
    team_members_file_name = node.get_parameter(
        'team_members_file_name').value
    if robots_spec_file_name.value == 'default_robots_spec':
        node.robots_spec_file_name = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'robots_spec', robots_spec_file_name.value)
    else:
        node.robots_spec_file_name = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'robots_spec', robots_spec_file_name.value)
    node.get_logger().info('Robots specification file used: '
                           + robots_spec_file_name.value)
    dict_type_robot = get_json_data(node.robots_spec_file_name)

    # Get mission spec package corresponding to use case
    team_members_file_name = node.get_parameter('team_members_file_name')
    if team_members_file_name.value == 'default_team_spec':
        node.team_members_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', team_members_file_name.value)
    else:
        node.team_members_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', team_members_file_name.value)
    node.get_logger().info('Team specification file used for robot specs: '
                           + team_members_file_name.value)
    dict_team = get_json_data(node.team_members_file_path)

    for robot_key in dict_team:
        dict_team[robot_key].update(
            dict_type_robot[dict_team[robot_key]["robot_type"]])

    return dict_team


def get_mission_spec_data(node, must_hmi_path=None):
    """
    Get mission spec parameter to find the spec file to use.
    Then load and return the data from the mission specs json file
    :param node: node to use to retrieve data
    :param must_hmi_path: str, path to the hmi json
    :return: dict with mission specs data
    """
    mission_spec_file_path = node.get_parameter('mission_spec_file_path')
    if mission_spec_file_path.value == 'default_mission_spec':
        node.mission_spec_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', mission_spec_file_path.value)
    else:
        node.mission_specs_file_path = os.path.join(
            get_package_share_directory('patrol_mission_spec'), 'data',
            'mission_spec', mission_spec_file_path.value)
    node.get_logger().info('Mission specification file used: '
                           + mission_spec_file_path.value)
    return get_json_data(node.mission_specs_file_path)


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
    except Exception:
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


def print_dict(d, indent=0):
    """
    Print a nested dict
    :param d: dict to print
    :param indent: indent to use for pretty printing
    """
    if isinstance(d, dict):
        for key, value in d.items():
            print('\t' * indent + str(key))
            if isinstance(value, dict):
                print_dict(value, indent+1)
            else:
                print('\t' * (indent+1) + str(value))
    else:
        print('ERROR, not a dict')


def add_row_to_csv(dir, file, data):
    """Adds a row to a csv file.
    :param dir: String, the path to the directory containing the target file.
    :param file: String, the name of the target csv file.
    :param data: list,  the row to be added into the csv file.
    """
    if not os.path.exists(dir):
        os.makedirs(dir)
    file_path = dir + file
    with open(file_path, mode='a+') as outfile:
        csv_writer = csv.writer(
            outfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        csv_writer.writerow(data)


def euclidean_distance_2d(p_a, p_b):
    return sqrt(pow((p_a.x - p_b.x), 2) + pow((p_a.y - p_b.y), 2))


def euclidean_distance_3d(p_a, p_b):
    return sqrt(pow((p_a.x - p_b.X), 2)
                + pow((p_a.y - p_b.y), 2)
                + pow((p_a.z - p_b.z), 2))


class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def get_norm(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))

    def set_vector(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def get_vector_3d(p_a, p_b):
    x = p_b.x - p_a.x
    y = p_b.y - p_a.y
    z = p_b.z - p_a.z
    return Vector3D(x, y, z)


# patrol UTILS FUNCTIONS
# TODO moves these functions in another file to split properly tools
def fill_header(sender_id, addressees=None, message_id=None,
                communication_time=None, required_acknowledgement=False):
    """
    Fill a patrol Header message
    :param sender_id:
    :param addressees:
    :param message_id:
    :param communication_time:
    :param required_acknowledgement:
    :return:
    """
    msg = Header()
    msg.sender_id = sender_id
    if addressees:
        msg.addressees = addressees
    if message_id:
        msg.message_id = message_id
    else:
        msg.message_id = generate_message_id(None)
    if communication_time:
        msg.communication_time = communication_time
    else:
        msg.communication_time = generate_communication_time(None)
    msg.required_acknowledgement = required_acknowledgement
    return msg


def generate_message_id(node, cpt=None):
    """
    Generate an ID for a message
    :param node:
    :param cpt:
    :return:
    """
    # TODO Use uuid
    return "msg_id_XX"


def generate_communication_time(node):
    """
    Return current time for a communication time attribute of a message
    :param node:
    :return:
    """
    # TODO
    return Time()


def fill_award(bid_st_msg, must_report=False):
    """
    This functions fills a patrol award message
    :param bid_st_msg: bid stamped message corresponding to the award
    :param must_report: boolean to indict if winner has to report after
    execution of items
    :return: award message
    """
    award = Award()
    award.bidder_id = bid_st_msg.header.sender_id
    award.bundle_id = bid_st_msg.body_msg.bundle_description.bundle_id
    award.must_report = must_report
    return award
