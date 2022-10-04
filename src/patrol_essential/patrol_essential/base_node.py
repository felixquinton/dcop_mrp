import rclpy

# Messages for patrol auction processes
from caf_messages.msg import (AnnouncementStamped, BidStamped, AuctionHeader,
                              BundleDescription, ItemDescription, Header,
                              ResultsStamped, AwardAcknowledgementStamped)

from patrol_messages.msg import LastVisitsStamped

# In ROS2 there isn't yet a callback_args to pass args to a listener_cb, the
# same can be done to ActionServer cbs partial func allow it
from caf_essential.base_node import BaseNode as CafBaseNode

# useful imports to call several tools
from patrol_essential.utils import utils

auctions_msg_types = [AnnouncementStamped, BidStamped, AuctionHeader,
                      BundleDescription, ItemDescription, Header,
                      ResultsStamped, AwardAcknowledgementStamped]

patrol_msg_types = [LastVisitsStamped]


class BaseNode(CafBaseNode):
    """
    Mother class for all patrol nodes.
    This an important class. Inside we retrieve statics mission information
    useful for each nodes but we also create all ROS2 necessary elements.
    By example we define functions for publishers and subscribers.
    """

    def __init__(self, positions=None):
        super().__init__(positions)


def main(args=None):
    rclpy.init(args=args)

    base_node = BaseNode()

    rclpy.spin(base_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
