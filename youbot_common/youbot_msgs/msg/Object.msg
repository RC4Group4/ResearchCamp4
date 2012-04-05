# The pose where the object can be found
geometry_msgs/PoseStamped pose

# The dimensions of the object
geometry_msgs/Vector3Stamped dimensions

# Name of the object determined e.g. by the recognition
string name

# Extracted RGB image of the image
sensor_msgs/Image rgb_image

# Segmented cluster that represents the object
sensor_msgs/PointCloud2 cluster
