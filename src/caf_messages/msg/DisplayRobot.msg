# caf_messages/DiplayRobot.msg
# This message allows the sim_robots node to send position, energy and path data
# to the interface for a particular robot.

# Unique robot id
string robot_id

# Position of the robot
Pose pose

# Energy of the robot
float64 energy

# Path of the robot.
# See https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Path.msg
Path path
