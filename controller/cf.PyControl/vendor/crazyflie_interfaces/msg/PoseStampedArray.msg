# An array of stamped poses with a header
# The global header will store the time at which the poses were computed (not to be confused with the stamps of the poses themselves)

# This message type was added to common_interfaces in https://github.com/ros2/common_interfaces/pull/262
# Because it is usefull to this project it is added here. When upgrading in future this can be removed. 

std_msgs/Header header
geometry_msgs/PoseStamped[] poses