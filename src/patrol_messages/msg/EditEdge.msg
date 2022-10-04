# patrol_messages/EditEdge.msg

# Simulation time
float64 simulation_time

# Weight of the edge, useful if it must be modified
float64 weight

# Indicates if the edge is directed
bool directed

# Source and target of the node (any order will do if edge is undirected)
int16 source
int16 target

# Type of editing
# "remove": remove the edge from a robot's graph
string edit_type

# Types of robots concerned by the edge modification
string[] robot_types
