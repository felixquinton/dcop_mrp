from caf_essential.auction import BaseAuction as CafBaseAuction


class BaseAuction(CafBaseAuction):
    """
    Base class for auctions schemes
    """

    def __init__(self, auction_scheme_name, robot_node):
        super().__init__(auction_scheme_name, robot_node)
