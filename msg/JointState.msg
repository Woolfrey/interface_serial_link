# This is distinct from sensor_msgs/msg/JointState since a reference trajectory is defined by acceleration, not effort.

std_msgs/Header header                  # Contains a timestamp
string[] joint_name                     # Unique identifier
float64[] position                      # rad/m
float64[] velocity                      # rad/s or m/s
float64[] acceleration                  # rad/s/s or m/s/s
float64[] effort                        # Nm or N
