from caf_essential.utils import items_handlers
from caf_essential.utils import utils
from caf_messages.msg import Results, ResultsStamped


class BaseAuction:
    """
    Base class for auctions schemes
    """

    def __init__(self, auction_scheme_name, robot_node):
        self.auction_scheme_name = auction_scheme_name
        self.robot_node = robot_node
        self.current_auction_auctioneer = None
        self.auction_awaiting_acknowledgements = None

    def current_auction_auctioneer_timer_cb(self):
        """
        Main auctioneer callback.
        """
        pass

    def announcement_cb(self, msg, *args, **kwargs):
        """
        Announcement callback
        :param msg: received message
        """
        pass

    def bid_cb(self, msg, *args, **kwargs):
        """
        Bid callback
        :param msg: received message
        """
        pass

    def results_cb(self, msg, *args, **kwargs):
        """
        Results callback
        :param msg: received message
        """
        pass

    def send_acknowledgement(self, results_st_msg, bundle_id):
        """
        Sends acknowledgement to teammates
        :param results_st_msg: received results stamped message
        :param bundle_id: bundle if concerned by the acknowledgement
        """
        pass

    def auctioneer_self_award(self, msg, best_bid_val, bundle_id):
        """
        Method for the auctioneer to award items to itself.
        :param msg: the result message sent to the team
        :param best_bid_val: the auctioneer's bid for the item
        :param bundle_id: string, the id of the corresponding bundle.
        """
        pass

    def auctioneer_self_acknowledgment(self, bundle_id, auction_id,
                                       recall_announcement_st_msg):
        """
        Method for the auctioneer to acknowledge the allocation of items to
        itself.
        :param bundle_id: string, the id of the corresponding bundle.
        :param auction_id: the auction's id
        :param recall_announcement_st_msg: the stamped announcement message for
        the auction
        """
        pass

    def award_acknowledgement_cb(self, msg, *args, **kwargs):
        """
        Award Acknowledgement callback
        :param msg: received message
        """
        pass

    def awaiting_acknowledgements_timer_cb(self):
        """
        Awaiting acknowledgement messages callback
        """
        pass

    def send_announcement(self, *args, **kwargs):
        """
        Initiates an auction.
        """
        pass

    def compute_bid(self, *args, **kwargs):
        """
        Function used to calculate a bid
        """
        pass

    def solve_wdp(self, *args, **kwargs):
        """
        Solves a winner determination problem
        """
        pass

    def formulate_bids(self, msg):
        """
        Build a BidStamped msg to broadcast a robot's bid to the team.
        """
        pass

    def parse_and_store_auctions_items(self, msg):
        """
        This function parse items in auction announcement, create a specific
        item handler with each and store it. To use your specific use case you
        need to change the called return_item_handler function
        :param msg:
        :return:
        """
        self.robot_node.current_auction_bidder['items_handlers'] = {}

        item_list = msg.item_list
        for item in item_list:
            # Change this line for your use case
            item_hdl = items_handlers.return_item_handler(item)
            if item_hdl:
                self.robot_node.current_auction_bidder[
                    'items_handlers'][item_hdl.item_id] = item_hdl
            else:
                self.robot_node.get_logger().warn(
                    'Impossible to create an ItemHandler for this item')
                continue

    # TODO uniformize with parse_and_store_auctions_items for auctioneer case
    def auctioneer_parse_and_store_auctions_items(self, msg):
        """
        This function parse items in auction announcement, create a specific
        item handler with each and store it. To use your specific use case you
        need to change the called return_item_handler function
        :param announcement_msg:
        :return:
        """
        auction_id = msg.auction_header.auction_id
        self.robot_node.passed_auctions_bidder[
            auction_id]['items_handlers'] = {}
        item_list = msg.item_list
        for item in item_list:
            # Change this line for your use case
            item_hdl = items_handlers.return_item_handler(item)
            if item_hdl:
                self.robot_node.passed_auctions_bidder[
                    auction_id]['items_handlers'][item_hdl.item_id] = item_hdl
            else:
                self.robot_node.get_logger().warn(
                    'Impossible to create an ItemHandler for this item')
                continue

    def build_result_msg_from_winning_bids_st_msgs(self, winning_bids_st_msgs, close_round=True):
        """
        :param winning_bids_st_msgs:
        :param close_round:
        :return:
        """
        # Build results msg
        # Award[]
        award_list = []
        bundle_descriptions = []
        for winning_bid_st_msg in winning_bids_st_msgs:
            award_list.append(utils.fill_award(winning_bid_st_msg))
            # BundleDescription[]
            bundle_descriptions.append(winning_bid_st_msg.body_msg.bundle_description)

        # Results
        results_msg = Results()
        announcement_msg = self.current_auction_auctioneer['announcement_st_msg'].body_msg
        results_msg.auction_header = announcement_msg.auction_header
        results_msg.award_list = award_list
        results_msg.bundle_descriptions = bundle_descriptions
        results_msg.close_round = close_round
        # ResultsStamped
        msg = ResultsStamped()
        msg.header = utils.fill_header(self.robot_node.robot_id)
        msg.body_msg = results_msg
        return msg
