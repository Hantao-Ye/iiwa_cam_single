# the robot name for control
string robot_name

# the waypoints robot would pass through
geometry_msgs/Pose[] robot_poses

# the dynamic feature of the robot
float64 robot_stiffness
float64 robot_speed

bool wait_execution_finished
bool first_as_start