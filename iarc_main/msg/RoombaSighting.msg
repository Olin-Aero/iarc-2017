# Header associated with source frame
Header header

# Roombas seen. TF frames should be empty, with visible_location holding
# the observed pose
Roomba[] data

# "magical" ID's associated with the Roombas, if AR tag tracking is being used.
# If non-empty, must be the same length as data
int32[] magical_ids

# The center of the area completely scanned by this detection event
geometry_msgs/PointStamped fov_center

# The radius of the circle scanned by this detection event
float64 fov_radius