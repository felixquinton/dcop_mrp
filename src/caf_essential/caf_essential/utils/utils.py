import os
import json
from ament_index_python.packages import get_package_share_directory

# Messages for caf auction processes
from caf_messages.msg import (AnnouncementStamped,
                              AuctionHeader,
                              Award,
                              AwardAcknowledgementStamped,
                              Bid, BidStamped,
                              BundleDescription, ItemDescription,
                              Header,
                              Results, ResultsStamped)

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


def get_dict_value_list_keys(d, keys, init=False, return_element='last_value'):
    """
    Return value of a dict given a list of keys
    :param d: dict
    :param keys: list of keys
    :param init: Boolean. If a key is missing will init value with another dict
    :param return_element: last element to return, can be the last value or the dict just before
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
    # Get mission spec package corresponding to use case
    node.declare_parameter('mission_spec_package_name', 'caf_mission_spec')
    mission_spec_package_name = node.get_parameter('mission_spec_package_name').value
    print('mission_spec_package_name')
    print(mission_spec_package_name)

    node.declare_parameter('team_members_file_name', 'default_team_spec.json')
    team_members_file_name = node.get_parameter('team_members_file_name')
    if team_members_file_name.value == 'default_team_spec':
        node.team_members_file_path = os.path.join(get_package_share_directory(mission_spec_package_name), 'data',
                                                   'mission_spec',
                                                   team_members_file_name.value)
    else:
        node.team_members_file_path = os.path.join(get_package_share_directory(mission_spec_package_name), 'data',
                                                   'current_spec',
                                                   team_members_file_name.value)
    node.get_logger().info('Team specification file used: ' + team_members_file_name.value)
    node.get_logger().info('mission spec package name: ' + mission_spec_package_name)
    return get_json_data(node.team_members_file_path)


def get_robots_spec_data(node):
    """
    Get team members parameter to find the spec file to use.
    Then load and return the data from the team member json file
    :param node: node to use to retrieve data
    :return: dict with robots spec data
    """
    mission_spec_package_name = node.get_parameter('mission_spec_package_name').value

    node.declare_parameter('robots_spec_file_name', 'default_robots_spec.json')
    robots_spec_file_name = node.get_parameter('robots_spec_file_name')
    if robots_spec_file_name.value == 'default_robots_spec':
        node.robots_spec_file_name = os.path.join(
            get_package_share_directory(mission_spec_package_name), 'data',
            'robots_spec', robots_spec_file_name.value)
    else:
        node.robots_spec_file_name = os.path.join(
            get_package_share_directory(mission_spec_package_name), 'data',
            'current_spec', robots_spec_file_name.value)
    node.get_logger().info('Robots specification file used: ' + robots_spec_file_name.value)
    return get_json_data(node.robots_spec_file_name)


def get_mission_spec_data(node):
    """
    Get mission spec parameter to find the spec file to use.
    Then load and return the data from the mission specs json file
    :param node: node to use to retrieve data
    :return: dict with mission specs data
    """
    node.declare_parameter('mission_spec_file_path',
                           'mission_spec_demo.json')
    mission_spec_file_path = node.get_parameter('mission_spec_file_path')
    if mission_spec_file_path.value == 'default_mission_spec':
        node.mission_spec_file_path = os.path.join(
            get_package_share_directory('caf_mission_spec'), 'data',
            'mission_spec', mission_spec_file_path.value)
    else:
        node.mission_specs_file_path = os.path.join(
            get_package_share_directory('caf_mission_spec'), 'data',
            'current_spec', mission_spec_file_path.value)
    node.get_logger().info('Mission specification file used: '
                           + mission_spec_file_path.value)
    return get_json_data(node.mission_specs_file_path)


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


def euclidean_distance_2d(p_a, p_b):
    return sqrt(pow((p_a.x - p_b.x), 2) + pow((p_a.y - p_b.y), 2))


def euclidean_distance_3d(p_a, p_b):
    return sqrt(pow((p_a.x - p_b.X), 2) + pow((p_a.y - p_b.y), 2) + pow((p_a.z - p_b.z), 2))


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


# CAF UTILS FUNCTIONS
# TODO moves these functions in another file to split properly tools
def fill_header(sender_id, addressees=None, message_id=None, communication_time=None, required_acknowledgement=False):
    """
    Fill a CAF Header message
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
    This functions fills a CAF award message
    :param bid_st_msg: bid stamped message corresponding to the award
    :param must_report: boolean to indict if winner has to report after execution of items
    :return: award message
    """
    award = Award()
    award.bidder_id = bid_st_msg.header.sender_id
    award.bundle_id = bid_st_msg.body_msg.bundle_description.bundle_id
    award.must_report = must_report
    return award


def stamp_bids_msgs(bids_msgs, bidder_id):
    # Stamp msgs
    bids_msgs_stamped = []
    for bid_msg in bids_msgs:
        bid_msg_stamped = BidStamped()
        bid_msg_stamped.header = fill_header(bidder_id)
        bid_msg_stamped.body_msg = bid_msg
        bids_msgs_stamped.append(bid_msg_stamped)
    return  bids_msgs_stamped
