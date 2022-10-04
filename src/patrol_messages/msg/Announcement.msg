# patrol_messages/Announcement.msg

# AuctionHeader of the message
AuctionHeader auction_header

# stamp on the message creation
builtin_interfaces/Time auction_deadline

# List of the items for sell
ItemDescription[] item_list

# id and parameters of the bid valuation function that will be used by the auctioneer for this auction
string bid_valuation_function

# List of minimal bids for each items
float64[] min_bid_list
