time last_seen
string frame_id
uint8 type

uint8 RED=0
uint8 GREEN=1
uint8 OBSTACLE=2

# The ROS timestamp of the estimate last time at which the Roomba
# started a periodic turn
time last_turn

geometry_msgs/PoseWithCovarianceStamped visible_location