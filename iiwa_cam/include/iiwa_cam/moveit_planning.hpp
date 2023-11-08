#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/GetMotionSequence.h>

#include <iiwa_cam/iiwa.hpp>
#include <iiwa_cam/moveit_construct.hpp>

#include <iiwa_cam/GeneralPlan.h>
#include <iiwa_cam/GeneralExecution.h>
#include <iiwa_cam/PublishToRviz.h>

namespace moveit_planning
{
    namespace constants
    {
        namespace damping
        {
            const static std::vector<float> DAMP(3, 0.7);
        }

        namespace workspace
        {
            const static std::string origin_link("world");

            const static std::string move_group_name("iiwa_arms");
        }

        namespace planning_time
        {
            constexpr double fast = 0.5;
            constexpr double slow = 1;
        }
    }

    struct service_response
    {
        bool success;
        std::string error_msg;
    };

    /**
     * @brief The robot handler proving the interface to get move_group pointer and controller pointer
     *
     */
    class iiwa_handler
    {
    private:
        std::string name_, move_group_name_, end_effector_name_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
        std::shared_ptr<cam::Kuka> controller_ptr_;

    public:
        iiwa_handler(const std::string &robot_name, const bool real_robot_execution) : name_(robot_name),
                                                                                       move_group_name_(robot_name + "_arm"),
                                                                                       end_effector_name_(robot_name + "_gripper_ee"),
                                                                                       move_group_ptr_(new moveit::planning_interface::MoveGroupInterface(move_group_name_))
        {
            if (real_robot_execution)
            {
                controller_ptr_ = std::make_shared<cam::Kuka>(robot_name);
            }
            else
            {
                controller_ptr_ = nullptr;
            }

            move_group_ptr_->setPlanningTime(constants::planning_time::fast);
            move_group_ptr_->setNumPlanningAttempts(10);
            move_group_ptr_->setReplanAttempts(10);
        };

        ~iiwa_handler(){};

        /**
         * @brief Get the robot name
         *
         * @return std::string name
         */
        std::string get_name()
        {
            return name_;
        }

        /**
         * @brief Get the move group name
         *
         * @return std::string move group name
         */
        std::string get_move_group_name()
        {
            return move_group_name_;
        }

        /**
         * @brief Get the end effector name
         *
         * @return std::string end effector name
         */
        std::string get_end_effector_name()
        {
            return end_effector_name_;
        }

        /**
         * @brief Get the move group ptr
         *
         * @return moveit::planning_interface::MoveGroupInterface* move group interface pointer
         */
        moveit::planning_interface::MoveGroupInterfacePtr get_move_group_ptr()
        {
            return move_group_ptr_;
        }

        /**
         * @brief Get the controller ptr
         *
         * @return cam::Kuka* controller pointer
         */
        std::shared_ptr<cam::Kuka> get_controller_ptr()
        {
            return controller_ptr_;
        }
    };

    /**
     * @brief The moveit constructor stores the useful handler for moveit planning
     */
    class moveit_constructor
    {
    private:
        ros::NodeHandlePtr nh_ptr_;
        ros::ServiceClient sequence_path_client_;

        moveit::core::RobotModelPtr robot_model_ptr_;
        moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_ptr_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr_;

        std::unordered_map<std::string, std::shared_ptr<iiwa_handler>> robot_handlers_;
        bool real_robot_execution_;

        geometry_msgs::Pose iiwa_green_home_pose, iiwa_blue_home_pose;

        bool whether_current_reached_goal_position(geometry_msgs::Point target_position, geometry_msgs::Point current_position, double tolerance)
        {
            if (abs(target_position.x - current_position.x) <= tolerance &&
                abs(target_position.y - current_position.y) <= tolerance &&
                abs(target_position.z - current_position.z) <= tolerance)
                return true;

            return false;
        }

        bool whether_current_reached_goal_orientation(geometry_msgs::Quaternion target_orientation, geometry_msgs::Quaternion current_orientation, double tolerance)
        {
            tf2::Quaternion target_quaternion, current_quaternion;

            tf2::convert(target_orientation, target_quaternion);
            tf2::convert(current_orientation, current_quaternion);

            double target_r, target_p, target_y;
            double current_r, current_p, current_y;
            tf2::Matrix3x3 target_rotation_matrix(target_quaternion);
            tf2::Matrix3x3 current_rotation_matrix(current_quaternion);

            target_rotation_matrix.getEulerYPR(target_y, target_p, target_r);
            current_rotation_matrix.getEulerYPR(current_y, current_p, current_r);

            if ((abs(target_r - current_r) <= tolerance || abs(abs(target_r - current_r) - 2 * M_PI) <= tolerance) &&
                (abs(target_p - current_p) <= tolerance || abs(abs(target_p - current_p) - 2 * M_PI) <= tolerance) &&
                (abs(target_y - current_y) <= tolerance || abs(abs(target_y - current_y) - 2 * M_PI) <= tolerance))
                return true;

            return false;
        }

        bool whether_current_reached_goal(geometry_msgs::Pose target, geometry_msgs::Pose current)
        {
            return whether_current_reached_goal_position(target.position, current.position, 0.03) &&
                   whether_current_reached_goal_orientation(target.orientation, current.orientation, 0.01);
        }

        bool whether_current_reached_goal(std::vector<double> target, std::vector<double> current, double tolerance)
        {
            for (int i = 0; i < target.size(); i++)
            {
                if (abs(target[i] - current[i]) > tolerance)
                    return false;
            }

            return true;
        }

        bool robot_execution(iiwa_cam::GeneralExecution::Request &req, iiwa_cam::GeneralExecution::Response &res)
        {
            auto robot_name = req.general_traj.robot_name;
            auto speed = (float)req.general_traj.robot_speed;
            auto stiffness = std::vector<float>(3, (float)req.general_traj.robot_stiffness);
            auto trajectory = req.general_traj.robot_trajectory;
            double long_term_efficient = 0.6;
            double v_threshold = 0.0005;

            if (real_robot_execution_)
            {
                robot_handlers_[robot_name]->get_controller_ptr()->exe_joint_traj(trajectory.joint_trajectory.points, speed, stiffness, constants::damping::DAMP, cam::Kuka::JOINT_SPLINE_MODE::CARTESIAN_IMPEDANCE_MODE);

                auto target_joint_values = req.general_traj.robot_trajectory.joint_trajectory.points.back().positions;
                auto current_joint_values = robot_handlers_[robot_name]->get_move_group_ptr()->getCurrentJointValues();

                std::vector<double> v_l = {10, 10, 10, 10, 10, 10, 10};
                while (req.general_traj.wait_execution_finished && !whether_current_reached_goal(target_joint_values, current_joint_values, 0.01))
                {
                    ros::Duration(0.1).sleep();
                    auto prev_joint_values = current_joint_values;
                    current_joint_values = robot_handlers_[robot_name]->get_move_group_ptr()->getCurrentJointValues();

                    std::vector<double> v_s = {0, 0, 0, 0, 0, 0, 0};
                    bool robot_moving = false;
                    for (size_t i = 0; i < v_l.size(); i++)
                    {
                        v_s[i] = abs((current_joint_values[i] - prev_joint_values[i]) / 0.1);
                        v_l[i] = v_l[i] * long_term_efficient + v_s[i] * (1 - long_term_efficient);
                        ROS_INFO_STREAM("v_l " << v_l[i]);
                        if (v_l[i] > v_threshold)
                        {
                            robot_moving = true;
                        }
                    }

                    ROS_INFO_STREAM("Target joint values are ");
                    for (auto value : target_joint_values)
                    {
                        std::cout << value << std::endl;
                    }

                    ROS_INFO_STREAM("Current joint values are ");
                    for (auto value : current_joint_values)
                    {
                        std::cout << value << std::endl;
                    }
                    if (robot_moving == false)
                    {
                        ROS_INFO_STREAM("Robot moving too slow");
                        break;
                    }
                }
            }
            else
            {
                robot_handlers_[robot_name]->get_move_group_ptr()->execute(trajectory);
            }

            return true;
        }

        bool robot_plan(iiwa_cam::GeneralPlan::Request &req, iiwa_cam::GeneralPlan::Response &res)
        {
            moveit_msgs::GetMotionSequenceRequest motion_sequence_req;
            moveit_msgs::GetMotionSequenceResponse motion_sequence_res;

            auto robot_name = req.general_control.robot_name;

            auto robot_move_group_ptr = robot_handlers_[robot_name]->get_move_group_ptr();
            auto robot_joint_group_ptr = robot_model_ptr_->getJointModelGroup(robot_handlers_[robot_name]->get_move_group_name());

            moveit::core::RobotStatePtr robot_state_ptr(robot_move_group_ptr->getCurrentState());
            auto target_poses = req.general_control.robot_poses;

            bool record_ik = false;

            if (req.general_control.first_as_start && !(robot_state_ptr->setFromIK(robot_joint_group_ptr, target_poses[0])))
            {
                res.error_msg = "Failed to solve inverse kinematics for robot " + robot_name + " for pose " + std::to_string(0);
                res.success = false;

                ROS_WARN_STREAM(res.error_msg);

                return true;
            }

            robot_move_group_ptr->setStartState(*robot_state_ptr);

            int start_index = req.general_control.first_as_start ? 1 : 0;

            for (int i = start_index; i < target_poses.size(); i++)
            {
                moveit_msgs::MotionSequenceItem motion_sequence_item;

                motion_sequence_item.blend_radius = 0.1;

                if (i == target_poses.size() - 1)
                    motion_sequence_item.blend_radius = 0.0;

                auto target_pose = target_poses[i];

                if (!(robot_state_ptr->setFromIK(robot_joint_group_ptr, target_pose)))
                {
                    res.error_msg = "Failed to solve inverse kinematics for robot " + robot_name + " for pose " + std::to_string(i) + " to pose " + std::to_string(i + 1);
                    res.success = false;

                    ROS_WARN_STREAM(res.error_msg);

                    return true;
                }

                robot_move_group_ptr->setJointValueTarget(*robot_state_ptr);
                robot_move_group_ptr->constructMotionPlanRequest(motion_sequence_item.req);

                if (i != start_index)
                    motion_sequence_item.req.start_state = moveit_msgs::RobotState();

                motion_sequence_req.request.items.push_back(motion_sequence_item);
            }

            sequence_path_client_.call(motion_sequence_req, motion_sequence_res);
            if (motion_sequence_res.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                res.error_msg = "Failed to generate trajectory, the error code in moveit is " + std::to_string(motion_sequence_res.response.error_code.val);
                res.success = false;

                ROS_WARN_STREAM(res.error_msg);

                return true;
            }

            ROS_INFO_STREAM("The trajectory point number is " << motion_sequence_res.response.planned_trajectories.back().joint_trajectory.points.size());
            publish_poses_and_trajectories(motion_sequence_res.response.planned_trajectories, target_poses, robot_name);
            if (res.general_traj.attach_pkg)
            {
                auto pkg_pose = req.general_control.pkg_pose;
                moveit_construct::add_package(pkg_pose, req.general_control.pkg_size, planning_scene_interface_ptr_.get(), constants::workspace::origin_link);
            }

            res.general_traj.robot_name = robot_name;

            res.general_traj.robot_trajectory = motion_sequence_res.response.planned_trajectories.front();
            res.general_traj.robot_poses = target_poses;

            res.general_traj.robot_speed = req.general_control.robot_speed;
            res.general_traj.robot_stiffness = req.general_control.robot_stiffness;

            res.general_traj.wait_execution_finished = req.general_control.wait_execution_finished;
            res.general_traj.first_as_start = req.general_control.first_as_start;

            res.general_traj.attach_pkg = req.general_control.attach_pkg;
            res.general_traj.detach_pkg = req.general_control.detach_pkg;
            res.general_traj.pkg_size = req.general_control.pkg_size;

            res.success = true;

            return true;
        }

        void set_planner(const std::string &robot_name, const std::string &planning_pipeline_id)
        {
            robot_handlers_[robot_name]->get_move_group_ptr()->setPlanningPipelineId(planning_pipeline_id);

            if (planning_pipeline_id.compare("pilz_industrial_motion_planner") == 0)
            {
                robot_handlers_[robot_name]->get_move_group_ptr()->setPlannerId("PTP");
            }
        }

    public:
        moveit_constructor(const std::vector<std::string> &robot_names, const moveit::core::RobotModelPtr robot_model, ros::NodeHandlePtr node_handle, bool real_robot_execution) : nh_ptr_(node_handle),
                                                                                                                                                                                    robot_model_ptr_(robot_model),
                                                                                                                                                                                    move_group_ptr_(new moveit::planning_interface::MoveGroupInterface(constants::workspace::move_group_name)),
                                                                                                                                                                                    planning_scene_interface_ptr_(new moveit::planning_interface::PlanningSceneInterface()),
                                                                                                                                                                                    visual_tools_ptr_(new moveit_visual_tools::MoveItVisualTools(constants::workspace::origin_link, rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model_ptr_)),
                                                                                                                                                                                    real_robot_execution_(real_robot_execution)
        {
            sequence_path_client_ = nh_ptr_->serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");

            visual_tools_ptr_->enableBatchPublishing();
            visual_tools_ptr_->deleteAllMarkers();
            visual_tools_ptr_->trigger();

            move_group_ptr_->setPlanningTime(constants::planning_time::fast);
            move_group_ptr_->setNumPlanningAttempts(10);

            for (auto name : robot_names)
            {
                robot_handlers_.insert(std::pair<std::string, std::shared_ptr<iiwa_handler>>(name, std::make_shared<iiwa_handler>(name, real_robot_execution_)));
                set_planner(name, "pilz_industrial_motion_planner");
            }
        }

        ~moveit_constructor(){};

        moveit::planning_interface::PlanningSceneInterfacePtr get_planning_scene_interface_ptr()
        {
            return planning_scene_interface_ptr_;
        }

        moveit_visual_tools::MoveItVisualToolsPtr get_visual_tools_ptr()
        {
            return visual_tools_ptr_;
        }

        /**
         * @brief Publish poses and trajectories line to rviz
         *
         * @param trajectories the robot trajectories
         * @param poses the target poses
         * @param robot_name the robot name
         */
        void publish_poses_and_trajectories(std::vector<moveit_msgs::RobotTrajectory> &trajectories, std::vector<geometry_msgs::Pose> &poses, const std::string &robot_name, const bool clean = true)
        {
            if (clean)
            {
                visual_tools_ptr_->deleteAllMarkers();
            }

            auto joint_group = robot_model_ptr_->getJointModelGroup(robot_handlers_[robot_name]->get_move_group_name());
            auto link_model = joint_group->getLinkModel(robot_handlers_[robot_name]->get_end_effector_name());

            for (int i = 0; i < poses.size(); i++)
            {
                visual_tools_ptr_->publishAxisLabeled(poses[i], "Pose goal " + std::to_string(i + 1));
            }

            for (int i = 0; i < trajectories.size(); i++)
            {
                visual_tools_ptr_->publishTrajectoryLine(trajectories[i], link_model, joint_group);
            }

            visual_tools_ptr_->trigger();
        }

        void publish_trajectory(moveit_msgs::RobotTrajectory trajectory, const std::string &robot_name, const bool clean = true)
        {
            if (clean)
            {
                visual_tools_ptr_->deleteAllMarkers();
            }

            auto joint_group = robot_model_ptr_->getJointModelGroup(robot_handlers_[robot_name]->get_move_group_name());
            auto link_model = joint_group->getLinkModel(robot_handlers_[robot_name]->get_end_effector_name());

            if (trajectory.joint_trajectory.points.size() != 0)
                visual_tools_ptr_->publishTrajectoryLine(trajectory, link_model, joint_group);

            visual_tools_ptr_->trigger();
        }

        bool general_execution_callback(iiwa_cam::GeneralExecution::Request &req, iiwa_cam::GeneralExecution::Response &res)
        {
            if (req.general_traj.robot_trajectory.joint_trajectory.points.size() == 0)
            {
                res.error_msg = "Empty trajectory message";
                res.success = false;

                return true;
            }

            return robot_execution(req, res);
        }

        bool general_plan_callback(iiwa_cam::GeneralPlan::Request &req, iiwa_cam::GeneralPlan::Response &res)
        {
            if (robot_handlers_.find(req.general_control.robot_name) == robot_handlers_.end() ||
                req.general_control.robot_poses.size() == 0 ||
                req.general_control.robot_speed <= 0 ||
                req.general_control.robot_speed > 1 ||
                req.general_control.robot_stiffness < 200 ||
                req.general_control.robot_stiffness > 3000)
            {
                res.error_msg = "The input data don't meet the requirement";
                res.success = false;

                ROS_INFO_STREAM(res.error_msg);

                return false;
            }

            return robot_plan(req, res);
        }

        bool publish_traj_to_rviz_callback(iiwa_cam::PublishToRviz::Request &req, iiwa_cam::PublishToRviz::Response &res)
        {
            if (req.general_traj.robot_trajectory.joint_trajectory.points.size() == 0)
            {
                res.success = true;
                return true;
            }

            publish_trajectory(req.general_traj.robot_trajectory, req.general_traj.robot_name, req.clean);

            res.success = true;

            return true;
        }

    };
}
