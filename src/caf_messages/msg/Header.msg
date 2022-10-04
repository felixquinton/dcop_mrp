# caf_messages/Header.msg

# Unique Id of the sender
string sender_id

# List of Id of the addressees
string[] addressees

# Unique message Id
string message_id

# stamp on the message creation
builtin_interfaces/Time communication_time

# True if a acknowledgement is required
bool required_acknowledgement




