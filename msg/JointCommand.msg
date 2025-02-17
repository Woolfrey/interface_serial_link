
std_msgs/Header header                      # Contains a time stamp
string[] name                               # Unique identifier for the joint
float64[] command                           # Vector for control input to joints
int32 mode                                  # Enumeration for type of command
int32 UNKNOWN  = 0                          #
int32 POSITION = 1                          # rad or m
int32 VELOCITY = 2                          # rad/s or m/s
int32 EFFORT   = 3                          # Nm or N
