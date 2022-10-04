import rosidl_runtime_py
from rosidl_parser.definition import NamespacedType


def get_action_type(action_name, package_name):
    """return action type for a given action name and a package"""
    try:
        return rosidl_runtime_py.import_message_from_namespaced_type(NamespacedType([package_name, 'action'],
                                                                                    action_name))
    except:
        raise


