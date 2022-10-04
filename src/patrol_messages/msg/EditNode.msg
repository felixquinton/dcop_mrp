# patrol_messages/EditNode.msg

# Simulation time
float64 simulation_time

# ID of the robot that discovered the event
string discoverer_id

# Node's ID
int16 node_id

# Type of editing
# "edit_sensors": change the sensors necessary to survey the node.
string edit_type

# Types of sensors able to survey the node
string[] sensor_types
