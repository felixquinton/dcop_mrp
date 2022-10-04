# caf_messages/Results.msg

# AuctionHeader of the message
AuctionHeader auction_header

# List of tuples specifying rewards (bidder, bundle_id, must_report)
Award[] award_list

# List of bundle descriptions
BundleDescription[] bundle_descriptions

# Boolean to close the auction round (not used in SI auctions)
bool close_round
