#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <iiwa.hpp>
#include <iiwa_msgs/JointPosition.h>

static ros::Publisher joint_state_pub;

static std::vector<std::string> names{};
static std::vector<ros::Subscriber> joint_state_sub{};

void iiwa_joint_state_callback(const iiwa_msgs::JointPosition::ConstPtr& msg, const std::string& robot_name)
{
  sensor_msgs::JointState new_msg;
  new_msg.velocity.resize(7);
  new_msg.header.stamp = ros::Time().now();

  for (int i = 1; i <= 7; i++)
  {
    std::stringstream ss;
    ss << robot_name << "_joint_" << i;

    new_msg.name.push_back(ss.str());
  }
  cam::assign_position_to_vec(*msg, new_msg.position);

  joint_state_pub.publish(new_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_rviz_state_receiver_service");
  ros::NodeHandle nh;

  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

  for (int i = 1; i < argc; i++)
  {
    std::string robot_name = std::string(argv[i]);
    std::string topic_name = "/" + robot_name + "/state/JointPosition";

    ros::Subscriber sub = nh.subscribe<iiwa_msgs::JointPosition>(topic_name, 100, boost::bind(&iiwa_joint_state_callback, _1, boost::cref(robot_name)));
    joint_state_sub.push_back(sub);
  }

  ros::spin();
  return 0;
}
