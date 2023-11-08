#include <vector>
#include <unordered_map>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>

#include <iiwa.hpp>

const float velocity = 0.1;
const std::vector<float> damp(7, 0.7);
const std::vector<float> stiff(7, 1000);

static std::unordered_map<std::string, std::shared_ptr<cam::Kuka>> robots;

std::string get_robot_name(ros::V_string joint_names)
{
  std::string joint = joint_names.at(0);
  const std::string index = "_joint";

  std::size_t found = joint.find(index);
  if (found != std::string::npos)
  {
    return std::string(joint.begin(), joint.begin() + found);
  }

  return {};
}

void moveit_callback(const moveit_msgs::ExecuteTrajectoryActionGoal &msg)
{
  const auto &traj = msg.goal.trajectory;

  std::string robot_name = get_robot_name(traj.joint_trajectory.joint_names);
  if (robot_name == "")
  {
    return;
  }

  robots[robot_name]->exe_joint_traj(traj, velocity, stiff, damp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_rviz_exec_service");
  ros::NodeHandle nh;

  for (int i = 1; i < argc; i++)
    robots.insert(std::pair<std::string, std::shared_ptr<cam::Kuka>>(argv[i], std::make_shared<cam::Kuka>(argv[i])));

  ros::Subscriber moveit_sub = nh.subscribe("/execute_trajectory/goal", 10, moveit_callback);

  ros::Duration(2).sleep();

  ros::spin();
  return 0;
}
