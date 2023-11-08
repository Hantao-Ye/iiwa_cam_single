#include <string>
#include <vector>

#include <ros/ros.h>

#include <iiwa_cam/moveit_planning.hpp>
#include <iiwa_cam/moveit_construct.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_planning_demo");
    ros::NodeHandlePtr node_handle_ptr(new ros::NodeHandle);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::vector<std::string> robot_names{};
    for (int i = 1; i < argc; i++)
    {
        robot_names.push_back(std::string(argv[i]));
    }

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    bool real_robot_execution;
    node_handle_ptr->getParam("/moveit_planning_demo/real_robot_execution", real_robot_execution);

    moveit_planning::moveit_constructor moveit_handler(robot_names, robot_model_loader->getModel(), node_handle_ptr, real_robot_execution);

    moveit_construct::add_floor(moveit_planning::constants::workspace::origin_link, moveit_handler.get_planning_scene_interface_ptr().get(), 0.9, 1.5, 0, 0, "floor");

    moveit_handler.get_visual_tools_ptr()->trigger();

    ros::ServiceServer general_plan_service = node_handle_ptr->advertiseService("general_plan", &moveit_planning::moveit_constructor::general_plan_callback, &moveit_handler);
    ros::ServiceServer general_execution_service = node_handle_ptr->advertiseService("general_execution", &moveit_planning::moveit_constructor::general_execution_callback, &moveit_handler);
    ros::ServiceServer publish_traj_to_rviz_service = node_handle_ptr->advertiseService("publish_traj_to_rviz", &moveit_planning::moveit_constructor::publish_traj_to_rviz_callback, &moveit_handler);

    ros::waitForShutdown();
    return 0;
}