string robot_name

moveit_msgs/RobotTrajectory robot_trajectory
geometry_msgs/Pose[] robot_poses
float64 robot_stiffness
float64 robot_speed

bool wait_execution_finished
bool first_as_start

# whether the robot detach the package after the trajectory
bool detach_pkg
# whether the robot attach the package after the trajectory
bool attach_pkg

# the package pose and size
geometry_msgs/Point pkg_size