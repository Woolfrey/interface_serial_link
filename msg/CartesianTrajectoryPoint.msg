# Defines a point in time for a Cartesian trajectory in SE(3)

float64 time                        # Point in time along the trajectory
geometry_msgs/Pose  pose            # Desired position & orientation
geometry_msgs/Twist   twist         # Desired linear & angular velocity
geometry_msgs/Accel accel           # Desired linear & angular acceleration
int32 reference_frame               # How the pose is defined
int32 GLOBAL   = 0                  # Pose defined in base frame
int32 LOCAL    = 1                  # Pose defined in current endpoint frame
int32 RELATIVE = 2                  # Pose defined as offset from a previous pose, relative to base
