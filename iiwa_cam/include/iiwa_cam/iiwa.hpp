#pragma once
// ROS build-in headers
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/WrenchStamped.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/ros.h>

// iiwa_cam pkg defined srvs
#include <iiwa_cam/EndEffectorPose.h>
#include <iiwa_cam/PathRecorder.h>

// iiwa_stack_cam defined msgs srvs acts
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <iiwa_msgs/GetFrames.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointSpline.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>
#include <iiwa_msgs/TimeToDestination.h>

// csv2 lib for csv handling
#include <csv2/csv2.hpp>

// iiwa_cam header file
#include <tree_generator.hpp>

// C++ STL
#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
namespace cam
{
  static unsigned int cnt = 0;
  constexpr int UNDEFINED_STATUS = -1;
} // namespace cam

namespace cam
{

  /**
   * @brief Read cartesian trajectory data from a file specified by file_name,
   * solve the trajectory in the vector traj
   *
   * @param file
   * @param traj
   */
  void read_cart_traj(char const *file, std::vector<geometry_msgs::Pose> &traj, std::vector<int> &status)
  {
    std::ifstream fp(file);
    std::string line;
    getline(fp, line); // ignore the first line of csv file

    while (getline(fp, line))
    {
      std::vector<double> data_line;
      std::istringstream readstr(line);

      for (int j = 0; j < 7; j++)
      {
        std::string number;
        getline(readstr, number, ',');
        data_line.emplace_back(std::stod(number));
      }

      geometry_msgs::Pose pose;
      pose.position.x = data_line.at(0);
      pose.position.y = data_line.at(1);
      pose.position.z = data_line.at(2);
      pose.orientation.w = data_line.at(3);
      pose.orientation.x = data_line.at(4);
      pose.orientation.y = data_line.at(5);
      pose.orientation.z = data_line.at(6);

      traj.emplace_back(pose);

      std::string number;
      getline(readstr, number, ',');
      status.emplace_back(std::stoi(number));
    }
  }

  /**
   * @brief Get input from the keyboard and continue the work
   */
  inline void press_to_go()
  {
    std::cout << "break point " << ++cnt << ": Press Enter to continue" << std::endl;

    char c;
    while ((c = getchar()) != EOF)
    {
      break;
    }
  }

  inline void print_vec(const std::vector<double> &vec)
  {
    for (auto it = vec.begin(); it != vec.end(); it++)
    {
      std::cout << *it;

      if (std::next(it) != vec.end())
        std::cout << ", ";
      else
        std::cout << std::endl;
    }
  }

  inline void assign_position_to_vec(const iiwa_msgs::JointPosition &msg,
                                     std::vector<double> &vec)
  {
    vec.resize(7);
    vec.at(0) = msg.position.a1;
    vec.at(1) = msg.position.a2;
    vec.at(2) = msg.position.a3;
    vec.at(3) = msg.position.a4;
    vec.at(4) = msg.position.a5;
    vec.at(5) = msg.position.a6;
    vec.at(6) = msg.position.a7;
  }
} // namespace cam

namespace cam
{

  class Kuka;

  class Kuka
  {
  public:
    enum JOINT_SPLINE_MODE
    {
      POSITION_CONTROL_MODE = 0,
      JOINT_IMPEDANCE_MODE,
      CARTESIAN_IMPEDANCE_MODE
    };

    class EndEffectorState
    {
      friend class Kuka;

    private:
      ros::ServiceClient ee_recorder_client;
      ros::ServiceClient ee_pose_client;
      std::string name;

      std::atomic<bool> watchdog_state;

      void watchdog()
      {
        while (watchdog_state && ros::ok())
        {
          ros::Duration(4).sleep();

          iiwa_cam::PathRecorder msg;
          msg.request.record = true;
          msg.request.robot_name = name;
          msg.request.watchdog = true;

          if (!watchdog_state)
            return;
          ee_recorder_client.call(msg);
          // std::cout << "client feeds dog" << std::endl;
        }
        // std::cout << "watchdog_state: " << std::boolalpha << watchdog_state
        //           << " client stop feeding" << std::endl;
      }

    public:
      EndEffectorState() = default;

      EndEffectorState(const std::string &rob_name) : name(rob_name)
      {
        ros::NodeHandle nh;
        ee_recorder_client = nh.serviceClient<iiwa_cam::PathRecorder>("/cam/iiwa/PathRecorder");

        ee_pose_client = nh.serviceClient<iiwa_cam::EndEffectorPose>("/cam/iiwa/EndEffectorPose");
      }

      ~EndEffectorState()
      {
        if (watchdog_state)
          end_recording();
      }

      std::pair<geometry_msgs::Pose, int> get_cart_pose()
      {
        iiwa_cam::EndEffectorPose msg;
        msg.request.robot_name = name;
        ee_pose_client.call(msg);
        if (!msg.response.success)
          std::cerr << msg.response.error << std::endl;
        return std::make_pair(msg.response.pose, msg.response.status);
      };

      void start_recording(bool watch_dog = true)
      {
        static int try_time = 3;
        while (--try_time >= 0)
        {
          if (ee_recorder_client.exists())
            break;
          std::cout << "Failed to connect to the end_effector_state_service\n"
                    << "Did you run \"rosrun iiwa_cam end_effector_state_service "
                       "[robot1 name] [robot2 name] ...\"?\n"
                    << std::endl;
        }

        static iiwa_cam::PathRecorder msg;
        msg.request.record = true;
        msg.request.robot_name = name;
        msg.request.watchdog = watch_dog;
        ee_recorder_client.call(msg);
        if (!msg.response.success)
          std::cerr << msg.response.error << std::endl;

        // start client watchdog
        if (watch_dog)
        {
          watchdog_state = true;
          std::thread(std::bind(&EndEffectorState::watchdog, this)).detach();
        }
      }

      void end_recording()
      {
        iiwa_cam::PathRecorder msg;
        msg.request.record = false;
        msg.request.robot_name = name;
        // msg.request.watchdog = watchdog_state;
        ee_recorder_client.call(msg);
        // stop client watchdog
        watchdog_state = false;
      }
    };

  private:
    ros::ServiceClient cart_spline_vel_client;
    ros::ServiceClient joint_vel_client;
    ros::ServiceClient ss_joint_vel_client;
    ros::ServiceClient ss_lin_vel_client;
    ros::ServiceClient remaining_time_client;

    ros::ServiceClient frames_client;

    ros::Publisher joint_spline_pub;
    ros::Publisher joint_ptp_droppable_pub;
    ros::Publisher cart_ptp_droppable_pub;
    ros::Publisher cart_lin_droppable_pub;

    actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> *joint_pos_client;

    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> *cartesian_pos_ptp_client;

    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> *cartesian_pos_lin_client;

    actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> *cartesian_spline_client;

    double m_joint_vel = 0;
    double m_joint_acc = 0;

    double m_cart_pos_vel = 0;
    double m_cart_ori_vel = 0;
    double m_cart_pos_acc = 0;
    double m_cart_ori_acc = 0;
    double m_cart_pos_jerk = 0;
    double m_cart_ori_jerk = 0;

    double m_joint_vel_drop = 0;
    double m_joint_acc_drop = 0;
    double m_joint_over_acc_drop = 0;

    double m_ss_vel_lin_drop = 0;
    double m_ss_acc_lin_drop = 0;
    double m_ss_over_acc_lin_drop = 0;

    bool m_print_info = true;

    std::string iiwa_name;

    EndEffectorState ee_state;

  private:
    iiwa_msgs::MoveToCartesianPoseAction build_cart_act(const geometry_msgs::Pose &pose, int status = UNDEFINED_STATUS,
                                                        double e1 = 0.0)
    {
      // action msg definition
      iiwa_msgs::MoveToCartesianPoseAction cartesian_pos_act;

      // set goal
      auto &poseStamped = cartesian_pos_act.action_goal.goal.cartesian_pose.poseStamped;

      if (status == UNDEFINED_STATUS)
      {
        status = 2;
        if (pose.position.x < 0 && std::abs(pose.position.y) < 0.02)
          status = 5;
      }

      cartesian_pos_act.action_goal.goal.cartesian_pose.redundancy.status = status;
      cartesian_pos_act.action_goal.goal.cartesian_pose.redundancy.e1 = e1;
      // see pdf 391

      // set frame id (important!)
      poseStamped.header.frame_id = iiwa_name + "_link_0";

      // set cartesian position
      poseStamped.pose = pose;

      return cartesian_pos_act;
    }

    void rob_init(const std::string &rob_name)
    {
      ros::NodeHandle nh;

      std::stringstream ss("/");
      ss << rob_name;

      iiwa_name = std::move(ss.str());

      cart_spline_vel_client =
          nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>(iiwa_name + "/configuration/setPTPCartesianLimits");

      joint_vel_client =
          nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>(iiwa_name + "/configuration/setPTPJointLimits");

      ss_joint_vel_client =
          nh.serviceClient<iiwa_msgs::SetSmartServoJointSpeedLimits>(iiwa_name + "/configuration/setSmartServoLimits");

      ss_lin_vel_client =
          nh.serviceClient<iiwa_msgs::SetSmartServoLinSpeedLimits>(iiwa_name + "/configuration/setSmartServoLinLimits");

      remaining_time_client =
          nh.serviceClient<iiwa_msgs::TimeToDestination>(iiwa_name + "/state/timeToDestination");

      frames_client =
          nh.serviceClient<iiwa_msgs::GetFrames>(iiwa_name + "/configuration/GetFrames");

      joint_pos_client = new actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction>(
          iiwa_name + "/action/move_to_joint_position");

      cartesian_pos_ptp_client = new actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>(
          iiwa_name + "/action/move_to_cartesian_pose");

      cartesian_pos_lin_client = new actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>(
          iiwa_name + "/action/move_to_cartesian_pose_lin");

      cartesian_spline_client =
          new actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction>(iiwa_name + "/action/move_along_spline");

      joint_spline_pub = nh.advertise<iiwa_msgs::JointSpline>(iiwa_name + "/command/JointSpline", 1);

      joint_ptp_droppable_pub = nh.advertise<iiwa_msgs::JointPosition>(iiwa_name + "/command/JointPosition", 1);

      cart_ptp_droppable_pub = nh.advertise<geometry_msgs::PoseStamped>(iiwa_name + "/command/CartesianPose", 1);

      cart_lin_droppable_pub = nh.advertise<geometry_msgs::PoseStamped>(iiwa_name + "/command/CartesianPoseLin", 1);

      set_vel_acc();
      set_vel_acc_drop();
      set_vel_acc_lin_drop();
      set_cart_traj_vel_acc();
    }

    void fill_joint_spline_segments(iiwa_msgs::JointSpline &spline_msg,
                                    const std::vector<trajectory_msgs::JointTrajectoryPoint> &traj_vec)
    {
      unsigned traj_size = traj_vec.size();

      ROS_INFO("spline size:  %u", traj_size);

      spline_msg.segments.reserve(traj_size);

      auto traj_vec_iter = traj_vec.begin();
      auto traj_vec_end_iter = traj_vec.end();
      while (traj_vec_iter != traj_vec_end_iter)
      {
        iiwa_msgs::JointSplineSegment seg;

        seg.joint_angle.resize(traj_vec_iter->positions.size());
        std::copy(traj_vec_iter->positions.begin(), traj_vec_iter->positions.end(), seg.joint_angle.begin());

        spline_msg.segments.emplace_back(seg);
        traj_vec_iter++;
      }
    }

  public:
    Kuka(const std::string &rob_name) : ee_state(rob_name) { rob_init(rob_name); }

    Kuka() : Kuka("iiwa") {}

    ~Kuka()
    {
      delete joint_pos_client;
      delete cartesian_pos_lin_client;
      delete cartesian_pos_ptp_client;
      delete cartesian_spline_client;
    }

    /**
     * @brief Set the printer enabled status
     *
     * @param print set the parameter to true to print logger message
     */
    void set_printer(const bool &print) { m_print_info = print; }

    /**
     * @brief Get the End Effector State object of this kuka, which allows
     * recording path and getting the real time cartesian position
     *
     * @return EndEffectorState&
     */
    EndEffectorState &end_effector_state() { return ee_state; }

    /**
     * @brief Get the saved frames from teaching pendant, the direct children of
     * the world frame should be named as "P[number]", e.g., "P0", "P1" ~ "P99"
     *
     */
    KukaTreeNode *get_recorded_frames()
    {
      iiwa_msgs::GetFrames frame_msg;
      frames_client.call(frame_msg);
      // std::cout << "get " << frame_msg.response.frame_size << std::endl;

      std::vector<std::string> &frame_names = frame_msg.response.frame_name;
      std::vector<std::string> &abs_paths = frame_msg.response.parent_name;
      std::vector<iiwa_msgs::JointQuantity> &joint_quantities = frame_msg.response.joint_position;
      std::vector<geometry_msgs::Pose> &cart_world_positions = frame_msg.response.cart_world_position;

      std::vector<Frame *> frames;
      for (int i = 0; i < frame_msg.response.frame_size; i++)
      {
        auto new_frame = new Frame(7);
        auto &jq = joint_quantities[i];
        new_frame->set_joint_pos(std::vector<double>{jq.a1, jq.a2, jq.a3, jq.a4, jq.a5, jq.a6, jq.a7});

        new_frame->set_cart_pos(std::make_pair(cart_world_positions[i], std::stoi(frame_msg.response.status[i])));
        frames.emplace_back(new_frame);
      }

      KukaTreeNode *tree_root = generate_tree(frame_names, abs_paths, frames);
      return tree_root;
    }

    /**
     * @brief Set the velocity, acceleration, override acceleration(refer its
     * description in the KUKA manuals) of droppable move including
     * move_joint_ptp_drop(), move_cart_ptp_drop()
     *
     * @param vel
     * @param acc
     * @param override_acc
     */
    bool set_vel_acc_drop(const double vel = 0.1, const double acc = 0.1, const double override_acc = 1.0)
    {
      if (vel == m_joint_vel_drop && acc == m_joint_acc_drop && override_acc == m_joint_over_acc_drop)
      {
        ROS_WARN("Returning from the if statement in iiwa.hpp");
        return true;
      }
      else
      {
        m_joint_vel_drop = vel;
        m_joint_acc_drop = acc;
        m_joint_over_acc_drop = override_acc;
      }
      // service msg definition
      static iiwa_msgs::SetSmartServoJointSpeedLimits joint_vel_msg;

      // set joint velocity limit
      joint_vel_msg.request.joint_relative_velocity = vel;
      joint_vel_msg.request.joint_relative_acceleration = acc;
      joint_vel_msg.request.override_joint_acceleration = override_acc;

      ss_joint_vel_client.call(joint_vel_msg);

      if (m_print_info)
      {
        std::cout << "set droppable Joint Velocity to " << vel << " --> " << std::flush;
        std::cout << (joint_vel_msg.response.success ? "SUCCESSFUL" : "FAILED") << std::endl;
      }

      return joint_vel_msg.response.success;
    }

    /**
     * @brief Set the velocity, acceleration, override acceleration(refer its
     * description in the KUKA manuals) of droppable cartesian linear move:
     * move_cart_lin_drop()
     *
     * @param vel
     * @param acc
     * @param override_acc
     * @return true when succeed
     * @return false when failed
     */
    bool set_vel_acc_lin_drop(const double vel = 0.1, const double acc = 0.1, const double override_acc = 1.0)
    {
      if (vel == m_ss_vel_lin_drop && acc == m_ss_acc_lin_drop && override_acc == m_ss_over_acc_lin_drop)
      {
        return true;
      }
      else
      {
        m_ss_vel_lin_drop = vel;
        m_ss_acc_lin_drop = acc;
        m_ss_over_acc_lin_drop = override_acc;
      }
      // service msg definition
      static iiwa_msgs::SetSmartServoLinSpeedLimits ss_lin_vel_msg;

      // set joint velocity limit
      ss_lin_vel_msg.request.max_cartesian_velocity.linear.x = vel;
      ss_lin_vel_msg.request.max_cartesian_velocity.linear.y = acc;
      ss_lin_vel_msg.request.max_cartesian_velocity.linear.z = override_acc;

      ss_lin_vel_client.call(ss_lin_vel_msg);
      if (m_print_info)
      {
        std::cout << "set droppable Lin Velocity to " << vel << " --> " << std::flush;
        std::cout << (ss_lin_vel_msg.response.success ? "SUCCESSFUL" : "FAILED") << std::endl;
      }
      return ss_lin_vel_msg.response.success;
    }

    /**
     * @brief Set the velocity and acceleration of joint space. This function
     * controls the velocity and acceleration of move_joint_ptp(), move_cart_ptp()
     *
     * @param vel default = 0.1
     * @param acc default = 0.1
     */
    bool set_vel_acc(const double vel = 0.1, const double acc = 0.1)
    {
      if (vel == m_joint_vel && acc == m_joint_acc)
      {
        return true;
      }
      else
      {
        m_joint_vel = vel;
        m_joint_acc = acc;
      }
      // service msg definition
      static iiwa_msgs::SetPTPJointSpeedLimits joint_vel_msg;

      // set joint velocity limit
      joint_vel_msg.request.joint_relative_velocity = vel;
      joint_vel_msg.request.joint_relative_acceleration = acc;

      joint_vel_client.call(joint_vel_msg);
      if (m_print_info)
      {
        std::cout << "set Joint Velocity to " << vel << " --> " << std::flush;
        std::cout << (joint_vel_msg.response.success ? "SUCCESSFUL" : "FAILED") << std::endl;
      }

      return joint_vel_msg.response.success;
    }

    /**
     * @brief Set the velocity, acceleration, and jerk in cartesian space. This
     * function controls the velocity, acceleration, and jerk of exe_cart_traj()
     *
     * @param maxCartesianVelocity default = 0.1
     * @param maxOrientationVelocity default = 0.5
     * @param maxCartesianAcceleration default = 0.2
     * @param maxOrientationAcceleration default = 0.1
     * @param maxCartesianJerk default = -1
     * @param maxOrientationJerk default = -1
     */
    bool set_cart_traj_vel_acc(const double maxCartesianVelocity = 0.1, const double maxOrientationVelocity = 0.5,
                               const double maxCartesianAcceleration = 0.2, const double maxOrientationAcceleration = 0.1,
                               const double maxCartesianJerk = -1.0, const double maxOrientationJerk = -1.0)
    {
      if (m_cart_pos_vel == maxCartesianVelocity &&
          m_cart_ori_vel == maxOrientationVelocity &&
          m_cart_pos_acc == maxCartesianAcceleration &&
          m_cart_ori_acc == maxOrientationAcceleration &&
          m_cart_pos_jerk == maxCartesianJerk &&
          m_cart_ori_jerk == maxOrientationJerk)
      {
        return true;
      }
      else
      {
        m_cart_pos_vel = maxCartesianVelocity;
        m_cart_ori_vel = maxOrientationVelocity;
        m_cart_pos_acc = maxCartesianAcceleration;
        m_cart_ori_acc = maxOrientationAcceleration;
        m_cart_pos_jerk = maxCartesianJerk;
        m_cart_ori_jerk = maxOrientationJerk;
      }

      iiwa_msgs::SetPTPCartesianSpeedLimits cart_vel_msg;

      cart_vel_msg.request.maxCartesianVelocity = maxCartesianVelocity;
      cart_vel_msg.request.maxCartesianAcceleration = maxCartesianAcceleration;
      cart_vel_msg.request.maxCartesianJerk = maxCartesianJerk;
      cart_vel_msg.request.maxOrientationVelocity = maxOrientationVelocity;
      cart_vel_msg.request.maxOrientationAcceleration = maxOrientationAcceleration;
      cart_vel_msg.request.maxOrientationJerk = maxOrientationJerk;

      cart_spline_vel_client.call(cart_vel_msg);

      if (m_print_info)
        std::cout << "set Cartesian PTP limits: \n(" << maxCartesianVelocity << ", " << maxOrientationVelocity << ", "
                  << maxCartesianAcceleration << ", " << maxOrientationAcceleration << ", " << maxCartesianJerk << ", "
                  << maxOrientationJerk << ") --> " << (cart_vel_msg.response.success ? "SUCCESSFUL" : "FAILED")
                  << std::endl;
      return cart_vel_msg.response.success;
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by joint space goal. The
     * unit of joint position is radiant
     *
     * @param node KukaTreeNode*
     * @param sleep_time default = 500 ms
     */
    void move_joint_ptp(KukaTreeNode *node, const double sleep_time = 500.0)
    {
      move_joint_ptp(node->frame->get_joint_pos(), sleep_time);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by joint space goal. The
     * unit of joint position is radiant
     *
     * @param vec
     * @param sleep_time default = 500 ms
     */
    void move_joint_ptp(const std::vector<double> &vec, const double sleep_time = 500.0)
    {
      move_joint_ptp(vec.at(0), vec.at(1), vec.at(2), vec.at(3), vec.at(4), vec.at(5), vec.at(6), sleep_time);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by joint space goal. The
     * unit of joint position is radiant
     *
     * @param joint_positions j1 ~ j7
     * @param sleep_time default = 500 ms
     */
    void move_joint_ptp(const double &j1, const double &j2, const double &j3, const double &j4, const double &j5,
                        const double &j6, const double &j7, const double sleep_time = 500.0)
    {
      static iiwa_msgs::MoveToJointPositionAction joint_pos_act;
      auto &joint_pos = joint_pos_act.action_goal.goal.joint_position;
      // joint_pos.robotName = iiwa_name;
      joint_pos.position.a1 = j1;
      joint_pos.position.a2 = j2;
      joint_pos.position.a3 = j3;
      joint_pos.position.a4 = j4;
      joint_pos.position.a5 = j5;
      joint_pos.position.a6 = j6;
      joint_pos.position.a7 = j7;
      joint_pos_client->sendGoal(joint_pos_act.action_goal.goal);
      ros::Duration(sleep_time * 1e-3).sleep();
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by joint space goal. The
     * unit of joint position is radiant. The robot will abandon previous goal
     * when it receive a new goal, even when it is still executing.
     *
     * @param vec
     */
    void move_joint_ptp_drop(const std::vector<double> &vec)
    {
      move_joint_ptp_drop(vec.at(0), vec.at(1), vec.at(2), vec.at(3), vec.at(4), vec.at(5), vec.at(6));
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by joint space goal. The
     * unit of joint position is radiant. The robot will abandon previous goal
     * when it receive a new goal, even when it is still executing.
     *
     * @param joint_positions j1 ~ j7
     */
    void move_joint_ptp_drop(const double &j1, const double &j2, const double &j3, const double &j4, const double &j5,
                             const double &j6, const double &j7)
    {
      static iiwa_msgs::JointPosition joint_pos;
      // joint_pos.robotName = iiwa_name;
      joint_pos.position.a1 = j1;
      joint_pos.position.a2 = j2;
      joint_pos.position.a3 = j3;
      joint_pos.position.a4 = j4;
      joint_pos.position.a5 = j5;
      joint_pos.position.a6 = j6;
      joint_pos.position.a7 = j7;
      joint_ptp_droppable_pub.publish(joint_pos);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
     * unit of cartesian position is meter
     *
     * @param node KukaTreeNode*
     * @param sleep_time default = 500 ms
     */
    void move_cart_ptp(KukaTreeNode *node, const double sleep_time = 500.0)
    {
      auto &pair = node->frame->get_cartesian_pos();
      move_cart_ptp(pair.first, pair.second, sleep_time);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
     * unit of cartesian position is meter
     *
     * @param pose cartesian position X Y Z w x y z
     * @param status
     * @param sleep_time default = 500 ms
     */
    void move_cart_ptp(const geometry_msgs::Pose &pose, const int status = UNDEFINED_STATUS,
                       const double sleep_time = 500.0)
    {
      cartesian_pos_ptp_client->sendGoal(build_cart_act(pose, status).action_goal.goal);
      ros::Duration(sleep_time * 1e-3).sleep();
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
     * unit of cartesian position is meter
     *
     * @param pose cartesian position X Y Z w x y z
     * @param status
     * @param sleep_time default = 500 ms
     */
    void move_cart_ptp(const double &posX, const double &posY, const double &posZ, const double &oriW, const double &oriX,
                       const double &oriY, const double &oriZ, const int status = UNDEFINED_STATUS,
                       const double sleep_time = 500.0)
    {
      static geometry_msgs::Pose pose;
      pose.position.x = posX;
      pose.position.y = posY;
      pose.position.z = posZ;
      pose.orientation.w = oriW;
      pose.orientation.x = oriX;
      pose.orientation.y = oriY;
      pose.orientation.z = oriZ;
      move_cart_ptp(pose, status, sleep_time);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
     * unit of cartesian position is meter. The robot will abandon previous goal
     * when it receive a new goal, even when it is still executing.
     *
     * @param pose cartesian position X Y Z w x y z
     * @param sleep_time default = 500 ms
     */
    void move_cart_ptp_drop(const geometry_msgs::Pose &pose)
    {
      geometry_msgs::PoseStamped poseStamped;

      // set cartesian position
      poseStamped.pose = pose;
      poseStamped.header.frame_id = iiwa_name + "_link_0";

      cart_ptp_droppable_pub.publish(poseStamped);
    }

    /**
     * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
     * unit of cartesian position is meter. The robot will abandon previous goal
     * when it receive a new goal, even when it is still executing.
     *
     * @param pose cartesian position X Y Z w x y z
     * @param sleep_time default = 500 ms
     */
    void move_cart_ptp_drop(const double &posX, const double &posY, const double &posZ, const double &oriW,
                            const double &oriX, const double &oriY, const double &oriZ)
    {
      static geometry_msgs::Pose pose;
      pose.position.x = posX;
      pose.position.y = posY;
      pose.position.z = posZ;
      pose.orientation.w = oriW;
      pose.orientation.x = oriX;
      pose.orientation.y = oriY;
      pose.orientation.z = oriZ;
      move_cart_ptp_drop(pose);
    }

    /**
     * @brief Move kuka linearly (LIN) assigned by cartesian space goal.
     * The unit of cartesian position is meter
     *
     * @param pose
     * @param status
     * @param sleep_time default = 500 ms
     */
    void move_cart_lin(geometry_msgs::Pose &pose, const int status = UNDEFINED_STATUS, const double sleep_time = 500.0)
    {
      cartesian_pos_lin_client->sendGoal(build_cart_act(pose, status).action_goal.goal);

      ros::Duration(sleep_time * 1e-3).sleep();
    }

    /**
     * @brief Move kuka linearly (LIN) assigned by cartesian space goal. The
     * unit of cartesian position is meter
     *
     * @param status
     * @param sleep_time default = 500 ms
     */
    void move_cart_lin(const double &posX, const double &posY, const double &posZ, const double &oriW, const double &oriX,
                       const double &oriY, const double &oriZ, const int status = UNDEFINED_STATUS,
                       const double sleep_time = 500.0)
    {
      static geometry_msgs::Pose pose;
      pose.position.x = posX;
      pose.position.y = posY;
      pose.position.z = posZ;
      pose.orientation.w = oriW;
      pose.orientation.x = oriX;
      pose.orientation.y = oriY;
      pose.orientation.z = oriZ;

      move_cart_lin(pose, status, sleep_time);
    }

    /**
     * @brief Move kuka linearly (LIN) assigned by cartesian space goal. The
     * unit of cartesian position is meter. The robot will abandon previous goal
     * when it receive a new goal, even when it is still executing.
     *
     * @param pose cartesian position X Y Z w x y z
     */
    void move_cart_lin_drop(const geometry_msgs::Pose &pose)
    {
      geometry_msgs::PoseStamped poseStamped;

      // set cartesian position
      poseStamped.pose = pose;
      poseStamped.header.frame_id = iiwa_name + "_link_0";

      cart_lin_droppable_pub.publish(poseStamped);
    }

    /**
     * @brief Move robot along a trajectory in cartesian space, use
     * set_cart_traj_vel_acc() and set_vel_acc_drop () before this
     * method to set speed
     *
     * @param trajectory
     * @param status status of start point, default = -1
     */
    void exe_cart_traj(const std::vector<geometry_msgs::Pose> &trajectory, const std::vector<int> &status)
    {
      if (trajectory.size() != status.size())
      {
        std::cout << "Failed to execute cartesian trajectory, size of trajectory "
                     "and status should be same!"
                  << std::endl;
        return;
      }

      iiwa_msgs::MoveAlongSplineAction spline_act_msg;

      spline_act_msg.action_goal.header.frame_id = iiwa_name + "_link_0";

      auto &seg_vec = spline_act_msg.action_goal.goal.spline.segments;

      size_t traj_len = trajectory.size();

      seg_vec.reserve(traj_len);
      auto status_iter = status.begin();

      for (auto iter = trajectory.begin(); iter != trajectory.end(); iter++)
      {
        iiwa_msgs::SplineSegment seg;
        seg.type = iiwa_msgs::SplineSegment::SPL;

        seg.point.poseStamped.header.frame_id = iiwa_name + "_link_0";
        seg.point.redundancy.status = *status_iter;
        status_iter++;

        seg.point.poseStamped.pose = *iter;

        seg_vec.emplace_back(seg);
      }

      if (m_print_info)
        std::cout << "cartesian trajectory size: " << trajectory.size() << std::endl;

      move_cart_ptp(trajectory[0], status[0]);

      cartesian_spline_client->sendGoal(spline_act_msg.action_goal.goal);
    }

    /**
     * @brief Move robot along a trajectory in joint space with cartesian
     * impedance control
     *
     * @param trajectory
     * @param velocity joint relative speed, default = 0.1
     * @param stiff stiffness on X, Y, Z, default = 2000
     * @param damp damping on X, Y, Z, default = 0.7
     */
    void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory, const float velocity = 0.1,
                        const float stiffX = 2000, const float stiffY = 2000, const float stiffZ = 2000,
                        const float dampX = 0.7, const float dampY = 0.7, const float dampZ = 0.7)
    {
      std::vector<float> stiff{stiffX, stiffY, stiffZ};
      std::vector<float> damp{dampX, dampY, dampZ};
      exe_joint_traj(trajectory.joint_trajectory.points, velocity, stiff, damp, CARTESIAN_IMPEDANCE_MODE);
    }

    /**
     * @brief Move robot along a trajectory in joint space, using joint impedance
     * control mode
     *
     * @param trajectory
     * @param velocity joint relative speed
     * @param stiff stiffness on 7 joints
     * @param damp damping on 7 joints
     */
    void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory, const double velocity,
                        const std::vector<double> &stiff, const std::vector<double> &damp)
    {
      exe_joint_traj(trajectory.joint_trajectory.points, velocity, std::vector<float>(stiff.begin(), stiff.end()),
                     std::vector<float>(damp.begin(), damp.end()), JOINT_IMPEDANCE_MODE);
    }

    /**
     * @brief Move robot along a trajectory in joint space, using joint impedance
     * control mode
     *
     * @param trajectory
     * @param velocity joint relative speed
     * @param stiff stiffness on 7 joints
     * @param damp damping on 7 joints
     */
    void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory, const float velocity, std::vector<float> &&stiff,
                        std::vector<float> &&damp)
    {
      exe_joint_traj(trajectory.joint_trajectory.points, velocity, stiff, damp, JOINT_IMPEDANCE_MODE);
    }

    /**
     * @brief Move robot along a trajectory in joint space, using joint impedance
     * control mode
     *
     * @param trajectory
     * @param velocity joint relative speed
     * @param stiff stiffness on 7 joints
     * @param damp damping on 7 joints
     */
    void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory, const float velocity,
                        const std::vector<float> &stiff, const std::vector<float> &damp)
    {
      exe_joint_traj(trajectory.joint_trajectory.points, velocity, stiff, damp, JOINT_IMPEDANCE_MODE);
    }

    /**
     * @brief Move robot along a trajectory in joint space
     *
     * @param trajectory
     * @param velocity joint relative speed, default = 0.1
     * @param stiff stiffness, size = 3 (cartesian impedance) or 7 (joint impedance)
     * @param damp damping, size = 3 (cartesian impedance) or 7 (joint impedance)
     * @param mode 2: cartesian impedance  1: joint impedance  0: position control
     */
    void exe_joint_traj(const std::vector<trajectory_msgs::JointTrajectoryPoint> &trajectory, const float velocity,
                        const std::vector<float> &stiff, const std::vector<float> &damp,
                        JOINT_SPLINE_MODE mode = JOINT_SPLINE_MODE::POSITION_CONTROL_MODE)
    {
      iiwa_msgs::JointSpline spline_msg;

      switch (mode)
      {
      case JOINT_SPLINE_MODE::CARTESIAN_IMPEDANCE_MODE:
        spline_msg.cartesian_stiffness.resize(3);
        spline_msg.cartesian_damping.resize(3);
        std::copy(stiff.begin(), stiff.begin() + 3, spline_msg.cartesian_stiffness.begin());
        std::copy(damp.begin(), damp.begin() + 3, spline_msg.cartesian_damping.begin());
        break;

      case JOINT_SPLINE_MODE::JOINT_IMPEDANCE_MODE:
        spline_msg.joint_stiffness.resize(7);
        spline_msg.joint_damping.resize(7);
        std::copy(stiff.begin(), stiff.begin() + 7, spline_msg.joint_stiffness.begin());
        std::copy(damp.begin(), damp.begin() + 7, spline_msg.joint_damping.begin());
        break;

      case JOINT_SPLINE_MODE::POSITION_CONTROL_MODE:

        break;

      default:
        ROS_ERROR("Wrong joint spline control mode, requires 0 ~ 2, gets: %d", (int)mode);
        ROS_ERROR("Joint spline aborted.");
        return;
        break;
      }

      fill_joint_spline_segments(spline_msg, trajectory);

      spline_msg.speed = velocity;
      spline_msg.mode = mode;

      move_joint_ptp(trajectory.front().positions);

      joint_spline_pub.publish(spline_msg);
    }

    double get_execution_remaining_time()
    {
      iiwa_msgs::TimeToDestination srv;

      remaining_time_client.call(srv);

      return srv.response.remaining_time;
    }
  };

} // namespace cam