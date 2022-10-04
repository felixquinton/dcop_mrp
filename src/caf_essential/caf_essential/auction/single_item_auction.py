from caf_essential.utils import utils
from caf_messages.msg import (AnnouncementStamped, BidStamped, Bid,
                              BundleDescription, AwardAcknowledgement,
                              AwardAcknowledgementStamped,
                              Results, ResultsStamped)
from caf_essential.base_node import BundleHandlerBidder
from caf_essential.auction import base_auction as ba

import traceback


class SingleItemAuction(ba.BaseAuction):
    """
    This class define the single auction scheme
    """

    def __init__(self, robot_node):
        super(SingleItemAuction, self).__init__('single_item_auction',
                                                robot_node)
        self.auction_duration = .05
        self.timer_period = .0125

    def current_auction_auctioneer_timer_cb(self, forbidden_task_pos="last"):
        """
        Main auctioneer callback.
        :param forbidden_task_pos: String. If "last", forbidden task are placed
        at the end of the auctioneer_auction_to_start list. If "first",
        forbidden task are placed at the beginning of the
        auctioneer_auction_to_start list
        """
        # cheap fix for the multiple call of this timer despite
        # send_announcement being called only once...
        if self.current_auction_auctioneer is None:
            return
        # stop timer
        try:
            self.current_auction_auctioneer['timer'].cancel()
            self.robot_node.get_logger().info("Timer canceled")
            recall_announcement_st_msg = \
                self.current_auction_auctioneer['announcement_st_msg']
            announcement_msg = \
                self.current_auction_auctioneer[
                    'announcement_st_msg'].body_msg
            auction_id = announcement_msg.auction_header.auction_id

            # self.robot_node.get_logger().info(
            #     'DEADLINE reached for auction ' + auction_id)

            # WDP
            bids = self.current_auction_auctioneer['bids_st_msgs']
        except:
            self.robot_node.get_logger().error(
                "From data assignation in auctionee_timer_cb: "
                + traceback.format_exc())

        # Evaluate auctioneer's bid
        # Retrieve item list
        # item_list = announcement_msg.item_list

        # for item in item_list:
        # Computing bid
        # self.robot_node.auctioneer_bid = self.compute_bid(item)

        best_bidder, best_bid_val, best_bid_st_msg = self.solve_wdp(bids)

        if best_bid_st_msg is not None:
            # TODO use a dedicated function in BaseAuction to build Award messages
            # Build results msg
            # Award[]
            award = utils.fill_award(best_bid_st_msg)
            award_list = [award]

            # BundleDescription[]
            bundle_description = best_bid_st_msg.body_msg.bundle_description
            bundle_descriptions = [bundle_description]
            # Results
            results_msg = Results()
            results_msg.auction_header = announcement_msg.auction_header
            results_msg.award_list = award_list
            results_msg.bundle_descriptions = bundle_descriptions
            # ResultsStamped
            msg = ResultsStamped()
            msg.header = utils.fill_header(self.robot_node.robot_id)
            msg.body_msg = results_msg

            # Store auction to wait acknowledgements
            self.current_auction_auctioneer['bid_opening'] = False
            self.auction_awaiting_acknowledgements = \
                self.current_auction_auctioneer
            self.current_auction_auctioneer = None
            self.auction_awaiting_acknowledgements[
                'allocated_bundles_to_confirm'] = {}
            for award_msg in award_list:
                bundle_id = award_msg.bundle_id
                # TODO use bundle_description instead of award_msg ?
                # To keep trace of allocated tasks
                self.auction_awaiting_acknowledgements[
                    'allocated_bundles_to_confirm'][bundle_id] = {
                        'award_msg': award_msg, 'is_accepted': False}

            # Send results
            self.robot_node.send_msg_to_teammates(msg)

            # Process message for itself TODO a dedicated function for that
            for obj in award_list:
                if obj.bidder_id == self.robot_node.robot_id:
                    self.auctioneer_self_award(msg, best_bid_val, obj.bundle_id)
                    self.auctioneer_self_acknowledgment(
                        obj.bundle_id, auction_id, recall_announcement_st_msg)
            # This line introduces a bug and might have to be removed from CAF
            # self.robot_node.set_moving_flag(self.robot_node.robot_id+str(1))
            # Timer to wait awards acknowledgements
            timer_period = self.timer_period  # seconds
            self.auction_awaiting_acknowledgements['timer'] = \
                self.robot_node.create_timer(
                    timer_period,
                    self.awaiting_acknowledgements_timer_cb)

        else:
            self._no_valid_bids(forbidden_task_pos)

    def _no_valid_bids(self, forbidden_task_pos):
        """
        The actions to take if the auctioneer receives no valid bids.
        :param forbidden_task_pos: String, the position at which the task that
        received no auction must be placed in the auction_to_start list.
        """
        if forbidden_task_pos == "last":
            # The forbidden task is removed from the beginning and added
            # at the end of the auctions to start.
            self.robot_node.auctioneer_auction_to_start.append(
                self.robot_node.auctioneer_auction_to_start[0])
            self.robot_node.auctioneer_auction_to_start.pop(0)
        elif forbidden_task_pos == "first":
            # Doing nothing is equivalent to remove the task and put it
            # again at the start.
            pass
        else:
            # Otherwise, TBD
            pass

    def announcement_cb(self, msg, *args, **kwargs):
        """
        Announcement callback
        :param msg: received message
        """
        print("Received an announcement", msg)
        if msg.header.sender_id == self.robot_node.robot_id:
            return

        try:
            # Parse and store items data
            # self.parse_and_store_auctions_items(msg.body_msg)
            bids_msgs = self.formulate_bids(msg)
            # TODO formulate bids must be in the auction module
            # Stamp msgs
            bids_msgs_stamped = []
            for bid_msg in bids_msgs:
                bid_msg_stamped = BidStamped()
                bid_msg_stamped.header = utils.fill_header(
                    self.robot_node.robot_id)
                bid_msg_stamped.body_msg = bid_msg
                bids_msgs_stamped.append(bid_msg_stamped)
            self.robot_node.send_bids(bids_msgs_stamped)
        except:
            self.robot_node.get_logger().error(traceback.format_exc())

    def bid_cb(self, msg, *args, **kwargs):
        """
        Bid callback
        :param msg: received message
        """

        print("Bid message from auctions,", msg)
        if msg.header.sender_id == self.robot_node.robot_id:
            return
        auction_id = msg.body_msg.auction_header.auction_id
        try:
            current_auction_id = \
                self.current_auction_auctioneer[
                    'announcement_st_msg'].body_msg.auction_header.auction_id
        except TypeError:
            # self.robot_node.get_logger().info(
            #     'No current auction, cannot treat bid msg')
            return
        except KeyError:
            self.robot_node.get_logger().info(
                'Problem detected with reset of \
                robot.current_auction_auctioneer')
            raise
        if current_auction_id != auction_id:
            self.robot_node.get_logger().info(
                'Auction from bid msg does not match current auction id')
            return
        elif not self.current_auction_auctioneer['bid_opening']:
            self.robot_node.get_logger().info('Bidding session already closed')
            return

        # add bid to received bid for this auction
        self.current_auction_auctioneer['bids_st_msgs'].append(msg)

    def results_cb(self, msg, *args, **kwargs):
        """
        Results callback
        :param msg: received message
        """
        if msg.header.sender_id == self.robot_node.robot_id:
            return
        auction_id = msg.body_msg.auction_header.auction_id
        try:
            current_auction_id = \
                self.robot_node.current_auction_bidder[
                    'announcement_st_msg'].body_msg.auction_header.auction_id
        except TypeError:
            self.robot_node.get_logger().info(
                'No current auction, cannot treat results msg')
            return
        except KeyError:
            self.robot_node.get_logger().info(
                'Problem detected with reset of robot.current_auction_bidder')
            raise
        if current_auction_id != auction_id:
            self.robot_node.get_logger().info(
                'Auction from results msg does not match current auction id')
            return

        # Check if concerned by award
        results_msg = msg.body_msg
        robot_won_items = False
        for award in results_msg.award_list:
            if award.bidder_id == self.robot_node.robot_id:
                robot_won_items = True
                self.robot_node.get_logger().info(
                    'Received an award ' + '\n'
                    + str(award))

                # Store award in order to plan it
                try:
                    bid_msg = self.robot_node.current_auction_bidder[
                        'bids_msgs'][award.bundle_id]
                    bundle_hdl = BundleHandlerBidder(bid_msg)
                    self.robot_node.obtained_bundles[
                        award.bundle_id] = bundle_hdl
                    self.robot_node.robot_plan.add_items_from_bundle_hdl(bundle_hdl)
                except TypeError:
                    self.robot_node.get_logger().error(
                        'ERROR, type error on \
                        robot.current_auction_bidder dict')
                    raise
                except KeyError:
                    self.robot_node.get_logger().error(
                        'ERROR, key error on \
                        robot.current_auction_bidder dict')
                    raise
                except:
                    self.robot_node.get_logger().error(traceback.format_exc())

                # send acknowledgement to auctioneer
                self.send_acknowledgement(msg, award.bundle_id)

        # End corresponding pending auction
        # TODO manage auction tour
        self.robot_node.passed_auctions_bidder[
            current_auction_id] = self.robot_node.current_auction_bidder
        self.robot_node.current_auction_bidder = None

        # # use a timer to plan obtained bundle
        # # timer allows to wait for others bundle
        # if not self.robot_node.busy:
        #     self.robot_node.busy = robot_won_items
        #     self.robot_node.execute_bundles_callback()
        if robot_won_items:
            self.robot_node.execute_bundles_callback()

    def send_acknowledgement(self, results_st_msg, bundle_id):
        """
        Sends acknowledgement to teammates
        :param results_st_msg: received results stamped message
        :param bundle_id: bundle if concerned by the acknowledgement
        """
        results_msg = results_st_msg.body_msg
        # AwardAcknowledgement
        award_acknowledgement_msg = AwardAcknowledgement(
            auction_header=results_msg.auction_header,
            bundle_id=bundle_id, is_accepted=True)
        # AwardAcknowledgementStamped
        msg = AwardAcknowledgementStamped()
        msg.header = utils.fill_header(self.robot_node.robot_id)
        msg.body_msg = award_acknowledgement_msg

        # Send AwardAcknowledgementStamped
        self.robot_node.send_msg_to_teammates(msg)

    def auctioneer_self_award(self, msg, best_bid_val, bundle_id):
        results_msg = msg.body_msg
        for award in results_msg.award_list:
            self.robot_node.get_logger().info(
                'Self-awarding ' + '\n'
                + str(award))

        # Store award in order to plan it
        try:
            announcement_msg = \
                self.auction_awaiting_acknowledgements[
                    'announcement_st_msg'].body_msg
            bundle_description = BundleDescription()
            bundle_description.item_bundle = announcement_msg.item_list
            bundle_description.bundle_id = bundle_id
            self_bid_msg = Bid(auction_header=announcement_msg.auction_header,
                               bundle_description=bundle_description,
                               bid=[best_bid_val])
            bundle_hdl = BundleHandlerBidder(self_bid_msg)
            self.robot_node.obtained_bundles[award.bundle_id] = bundle_hdl
            self.robot_node.robot_plan.add_items_from_bundle_hdl(bundle_hdl)
        except TypeError:
            self.robot_node.get_logger().error(
              'ERROR, type error on self.current_auction_bidder dict')
            raise
        except KeyError:
            self.robot_node.get_logger().error(
              'ERROR, key error on self.current_auction_bidder dict')
            raise

        self.robot_node.execute_bundles_callback()

    def award_acknowledgement_cb(self, msg, *args, **kwargs):
        """
        Award Acknowledgement callback
        :param msg: received message
        """
        print("Award ack message from auctions,", msg)
        if msg.header.sender_id == self.robot_node.robot_id:
            return
        auction_id = msg.body_msg.auction_header.auction_id
        try:
            awaiting_auction_id = \
                self.auction_awaiting_acknowledgements[
                    'announcement_st_msg'].body_msg.auction_header.auction_id
        except TypeError:
            self.robot_node.get_logger().info(
                'No current auction, cannot treat award acknowledgement msg')
            return
        except KeyError:
            self.robot_node.get_logger().info(
                'Problem detected with reset of \
                robot.auction_awaiting_acknowledgements')
            raise
        if awaiting_auction_id != auction_id:
            self.robot_node.get_logger().info(
                'Auction from award acknowledgement msg does not match \
                current auction id')
            return

        # Need to note concerned bundle as accepted
        bundle_id = msg.body_msg.bundle_id
        is_accepted = msg.body_msg.is_accepted
        try:
            self.auction_awaiting_acknowledgements[
                'allocated_bundles_to_confirm'][bundle_id]['is_accepted'] = \
                is_accepted
        except KeyError:
            self.robot_node.get_logger().info(
                'bundle_id does not match an allocated_bundles_to_confirm')
            raise

    def auctioneer_self_acknowledgment(self, bundle_id, auction_id,
                                       recall_announcement_st_msg):
        # Need to note concerned bundle as accepted
        self.auction_awaiting_acknowledgements[
            'allocated_bundles_to_confirm'][bundle_id]['is_accepted'] = True

        self.robot_node.passed_auctions_bidder[auction_id] = {
            'announcement_st_msg': recall_announcement_st_msg}
        # self.auctioneer_parse_and_store_auctions_items(
        #     recall_announcement_st_msg.body_msg)

        self.robot_node.execute_bundles_callback()

    def awaiting_acknowledgements_timer_cb(self):
        """
        Callback use to check award acknowledgement
        """
        try:
            self.auction_awaiting_acknowledgements['timer'].cancel()

            # Verify all awards have been accepted
            for bundle_id in self.auction_awaiting_acknowledgements[
                    'allocated_bundles_to_confirm'].keys():
                elem = self.auction_awaiting_acknowledgements[
                    'allocated_bundles_to_confirm'][bundle_id]
                if not elem['is_accepted']:
                    #  TODO Do something if not accepted
                    pass

            # Delete auction from awaiting auction
            self.robot_node.passed_auctions_auctioneer.append(
                self.auction_awaiting_acknowledgements)
            self.auction_awaiting_acknowledgements = None

            # Start new SI auction
            self.robot_node.auctioneer_auction_to_start.pop(0)
            self.robot_node.auction_cpt += 1
            self.send_announcement(marker="awaiter")
        except:
            self.robot_node.get_logger().error("From acknowledgement_timer_cb: "
                                               + traceback.format_exc())


    def send_announcement(self, marker="unknown"):
        """
        Initiates a SI auction.
        """
        try:
            self.robot_node.get_logger().warn("Running send_annoucenement")
            item = self.robot_node.auctioneer_auction_to_start[0]
            # Create announcement stamped msg
            msg = AnnouncementStamped()
            # fill stamp
            msg.header = utils.fill_header(self.robot_node.robot_id)
            # fill auction header
            msg.body_msg.auction_header.auction_id = \
                'auction_'+self.robot_node.robot_id+'_'+str(
                    self.robot_node.auction_cpt)
            # Define deadline
            auction_duration = self.auction_duration  # seconds (int)
            msg.body_msg.auction_deadline = \
                self.robot_node.get_clock().now().to_msg()
            msg.body_msg.auction_deadline.nanosec += int(auction_duration*1e9)
            msg.body_msg.item_list = [item]
            # Store this auction for robot as "auctioneer" role
            self.current_auction_auctioneer = {
                'announcement_st_msg': msg, 'bids_st_msgs': [],
                'bid_opening': True}
            # publish msg
            self.robot_node.send_auction_msg(msg)

            # Create timer to end auction
            self.current_auction_auctioneer['timer'] = \
                self.robot_node.create_timer(
                auction_duration,
                self.current_auction_auctioneer_timer_cb,
                callback_group=self.robot_node.callback_group)
        except:
            self.robot_node.get_logger().error(traceback.format_exc())

    def compute_bid(self, item, eps=0.1):
        try:
            # TODO use generic function ?
            # Adapt to handle AbstractAction Items estimation
            item_data = eval(item.item_data)
            item_pos = (item_data["target_position"]["x"],
                        item_data["target_position"]["y"])
            if not len(self.robot_node.robot_plan.actions) == 0:
                # TODO exploit directly last planned state ?
                last_action = self.robot_node.robot_plan.get_last_action()
                last_pos = {"x": last_action.target_position.x,
                            "y": last_action.target_position.y}
                robot_goal = last_pos
            else:
                robot_goal = None
            # Convert the robot's pos in 2D to a node on the navigation graph.
            if robot_goal is None:
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    self.robot_node.position, eps)
                remaining_distance = 0
            else:
                # TODO Ensure that this doesnt cause issues when robot's path has
                # several nodes.
                waypoint = self.robot_node.nav_graph.convert_pos_into_node(
                    (robot_goal['x'], robot_goal['y']), eps)
                remaining_distance = (
                    (robot_goal['x']-self.robot_node.position[0])**2
                    + (robot_goal['y']-self.robot_node.position[1])**2)**.5
            item_node = self.robot_node.nav_graph.convert_pos_into_node(
                item_pos, eps)

            # Now use navGraph methods to compute the robot -> task distance.
            bid = (
                self.robot_node.path_lenghts[waypoint][item_node]
                + remaining_distance)
            return [float(bid)]
        except:
            print(self.robot_node.robot_plan.actions, last_action)
            self.robot_node.get_logger().error(traceback.format_exc())

    def formulate_bids(self, msg):
        """
        This function return bids to submit
        NEEDS to be surcharged, actually a dumb bid algo is used
        :param msg: received announcement message
        :return: bids messages to send
        """
        auction_header = msg.body_msg.auction_header

        # Retrieve item list
        item_list = msg.body_msg.item_list
        bids_msgs = []

        i = 0
        for item in item_list:
            bid_msg = Bid()
            bundle_desc = BundleDescription()
            bundle_desc.bundle_id = \
                self.robot_node.robot_id + '_bundle_' + str(i)
            bundle_desc.item_bundle = [item]
            # Computing bid
            bid_msg.bid = self.compute_bid(item)
            bid_msg.bundle_description = bundle_desc
            bid_msg.auction_header = auction_header

            bids_msgs.append(bid_msg)

            i += 1
        return bids_msgs

    def send_bid(self, auctioneer_id, robot_id, bid):
        # TODO use function in robot ?
        # Create a message with the bid
        # Request an action from the auctioneer ??
        pass

    def determine_winner(self, bids_dict):
        # TODO move here solve WPD single item function in robot ?
        return min(bids_dict, key=bids_dict.get)

    def solve_wdp(self, bids):
        """
        Basic WDP
        :param bids: bids to compare
        :param auctioneer_bid: float, the auctioneer's bid
        :return: best bidder and best bid msg stamped
        """
        best_bid_val = 1e6
        best_bid_st_msg = None
        # if no one bids, auctioneer has to do the task
        best_bidder = self.robot_node.robot_id
        for bid in bids:
            print("Bidder :", bid.header.sender_id,
                  " bids :", bid.body_msg.bid[0])
            bid_val = bid.body_msg.bid[0]
            if bid_val < best_bid_val:
                best_bid_val = bid_val
                best_bidder = bid.header.sender_id
                best_bid_st_msg = bid

        announcement_msg = self.current_auction_auctioneer[
            'announcement_st_msg']
        auctioneer_bid = self.compute_bid(
            announcement_msg.body_msg.item_list[0])

        print("Auctioneer :", self.robot_node.robot_id,
              " bids :", auctioneer_bid[0])
        if auctioneer_bid[0] < best_bid_val:
            best_bidder = self.robot_node.robot_id
            best_bid_val = auctioneer_bid[0]
            best_bid_st_msg = BidStamped()
            best_bid_st_msg.header = utils.fill_header(
                self.robot_node.robot_id)
            bid_msg = Bid()
            bundle_desc = BundleDescription()
            # TODO This must be improved to ensure that bundle id are unique.
            announcement_msg = self.current_auction_auctioneer[
                'announcement_st_msg']
            bundle_id = 0
            bundle_desc.bundle_id = self.robot_node.robot_id + '_bundle_' \
                + str(bundle_id)
            # The rest is ok.
            bundle_desc.item_bundle = announcement_msg.body_msg.item_list
            # Computing bid
            bid_msg.bid = auctioneer_bid
            bid_msg.bundle_description = bundle_desc
            bid_msg.auction_header = announcement_msg.body_msg.auction_header
            best_bid_st_msg.body_msg = bid_msg
        print("Winner :", best_bidder, " with bid :", best_bid_val)
        return best_bidder, best_bid_val, best_bid_st_msg
