"""
This file is useful to create handlers that manages items awarded in a bundle.
For your use case you can use ItemHandler as mother class and create your own handlers.
"""

import json

# Import for caf basic use case
from geometry_msgs.msg import Point
from caf_messages.action import GoTo, AbstractAction


class ItemHandler:
    """
    This class is a mother class to handle item awarded in a bundle. This is only for static information access and not
    planning.
    """
    def __init__(self, item_description_msg):
        self.item_id = item_description_msg.item_id
        self.item_name = item_description_msg.item_name
        self.item_type = item_description_msg.item_type
        self.item_data = item_description_msg.item_data

    def parse_item_description_data(self):
        """
        This function parse item_data and store it in specific attribute
        NEEDS to be surcharged
        """
        pass

    def create_goal_msg(self, goal_id='default_goal_id'):  # TODO to remove
        """
        This function create a goal_msg for this item handler. Useful if item is already a valid action
        for functional layer
        NEEDS to be surcharged
        """
        return None


def return_item_handler(item_description_msg):
    """
    Function that returns the corresponding item handler for an item description message
    :param item_description_msg: caf item description message
    :return: corresponding item handler
    """
    if item_description_msg.item_type == "AbstractAction":
        return AbstractActionItemHandler(item_description_msg)
    elif item_description_msg.item_type == "GoTo":
        return GoToItemHandler(item_description_msg)
    else:
        print('ERROR, impossible to find a matching ItemHandler')
        return


# TODO use import_module to build specific items handlers ?
# ------------------------- CAF ITEMS HANDLERS ---------------------------------------------------
class AbstractActionItemHandler(ItemHandler):
    """
    Class that handle an AbstractAction item
    """
    def __init__(self, item_description_msg):
        super().__init__(item_description_msg)

        # Important, initialize parameters with default values
        self.target_duration = 0.0
        # Call function to parse and store items data
        self.parse_item_description_data()

    def parse_item_description_data(self):
        # Use json.load() to convert item_data to json
        # item_data needs to be a valid json string
        d = json.loads(self.item_data)

        try:
            self.target_duration = d['target_duration']
        except KeyError:
            print('ERROR, impossible to parse ', self.item_type)
            return

    def create_goal_msg(self, goal_id='default_goal_id'):
        return AbstractAction.Goal(goal_id=goal_id, target_duration=self.target_duration)


class GoToItemHandler(ItemHandler):
    """
    Class that handle an GoToItemHandler item
    """
    def __init__(self, item_description_msg):
        super().__init__(item_description_msg)

        # Important, initialize parameters with default values
        self.target_position = Point()
        # Call function to parse and store items data
        self.parse_item_description_data()

    def parse_item_description_data(self):
        # Use json.load() to convert item_data to json
        # item_data needs to be a valid json string
        d = json.loads(self.item_data)

        try:
            target_position = d['target_position']
            self.target_position = Point(x=target_position['x'], y=target_position['y'], z=target_position['z'])
        except KeyError:
            print('ERROR, impossible to parse ', self.item_type)
            return

    def create_goal_msg(self, goal_id='default_goal_id'):
        return GoTo.Goal(goal_id=goal_id, target_position=self.target_position)


