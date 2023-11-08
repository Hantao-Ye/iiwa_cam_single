#pragma once
#include <string>
#include <vector>
#include <cmath>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>

#include <eigen3/Eigen/Dense>

namespace moveit_construct
{
    enum orientation
    {
        origin,
        y_plus,
        y_minus,
        x_minus,
        x_plus,
    };

    /**
     * @brief Convert degree to radiant
     *
     * @param degree 180 degree to -180 degree
     * @return double
     */
    inline double degree_to_rad(double degree)
    {
        return (degree / 180.0) * M_PI;
    }

    /**
     * @brief Parse the string with delimiter
     *
     * @param str the input string containing doubles
     * @param delimiter the delimiter string
     * @return std::vector<double>
     */
    std::vector<double> parse_string(const std::string &str, const std::string delimiter)
    {
        std::vector<double> output;

        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;

        while ((pos_end = str.find(delimiter, pos_start)) != std::string::npos)
        {
            token = str.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;

            output.push_back(std::stod(token));
        }

        output.push_back(std::stod(str.substr(pos_start)));

        return output;
    }

    geometry_msgs::Point string_to_point(const std::string &str, const std::string &delimiter)
    {
        geometry_msgs::Point p;
        std::vector<double> parsing_result = parse_string(str, delimiter);

        p.x = parsing_result[0];
        p.y = parsing_result[1];
        p.z = parsing_result[2];

        return p;
    }

    geometry_msgs::Quaternion string_to_quat(const std::string &str, const std::string &delimiter)
    {
        geometry_msgs::Quaternion q;
        std::vector<double> parsing_result = parse_string(str, delimiter);

        q.x = parsing_result[0];
        q.y = parsing_result[1];
        q.z = parsing_result[2];
        q.w = parsing_result[3];

        return q;
    }

    /**
     * @brief Calculate the translation for each panel
     *
     * @param container_size the vector containing container shape info [x, y, z, t]
     * @param o the orientation of the panel
     * @return the translation of the panel
     */
    Eigen::Vector3d container_panel_translation(const std::vector<double> &container_size, orientation o)
    {
        Eigen::Vector3d res;

        res[2] = container_size[2] / 2 + container_size[3] / 2;

        switch (o)
        {
        case y_plus:
            res[0] = 0;
            res[1] = container_size[1] / 2 + container_size[3] / 2;
            break;
        case y_minus:
            res[0] = 0;
            res[1] = -container_size[1] / 2 - container_size[3] / 2;
            break;
        case x_minus:
            res[0] = -container_size[3] / 2 - container_size[0] / 2;
            res[1] = 0;
            break;
        case x_plus:
            res[0] = container_size[3] / 2 + container_size[0] / 2;
            res[1] = 0;
            break;
        default:
            res[2] = 0;
        }

        return res;
    }

    /**
     * @brief Add container to the move group interface
     *
     * @param container_center the container center
     * @param container_size the container size [x, y, z, t]
     * @param workspace_origin_link link of origin
     * @param planning_scene_interface_ptr planning scene interface nullptr
     * @return the id of the collision object
     */
    std::string add_container(const Eigen::Vector3d &container_center_pos, const Eigen::Quaterniond &container_orientation, const std::vector<double> &container_size, std::string workspace_origin_link, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr)
    {
        moveit_msgs::CollisionObject container_object;

        std_msgs::ColorRGBA object_color;
        object_color.a = 0.5;
        object_color.r = 1;
        object_color.g = 1;
        object_color.b = 1;

        container_object.id = "container";
        container_object.header.frame_id = workspace_origin_link;
        container_object.operation = moveit_msgs::CollisionObject::ADD;

        container_object.primitives.resize(5);
        container_object.primitive_poses.resize(5);

        tf::pointEigenToMsg(container_center_pos, container_object.pose.position);
        tf::quaternionEigenToMsg(container_orientation, container_object.pose.orientation);

        for (int i = 0; i < 5; i++)
        {
            container_object.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
            container_object.primitives[i].dimensions.resize(3);
            container_object.primitives[i].dimensions[2] = container_size[3];

            if (i >= 3)
            {
                container_object.primitives[i].dimensions[0] = container_size[2];
                container_object.primitives[i].dimensions[1] = container_size[1] + 2 * container_size[3];

                tf::quaternionEigenToMsg(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) * Eigen::Quaterniond{1, 0, 0, 0}, container_object.primitive_poses[i].orientation);
            }
            else if (i >= 1)
            {
                container_object.primitives[i].dimensions[0] = container_size[0];
                container_object.primitives[i].dimensions[1] = container_size[2];

                tf::quaternionEigenToMsg(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::Quaterniond{1, 0, 0, 0}, container_object.primitive_poses[i].orientation);
            }
            else
            {
                container_object.primitives[i].dimensions[0] = container_size[0] + 2 * container_size[3];
                container_object.primitives[i].dimensions[1] = container_size[1] + 2 * container_size[3];

                container_object.primitive_poses[i].orientation.w = 1;
            }

            tf::pointEigenToMsg(container_panel_translation(container_size, orientation(i)), container_object.primitive_poses[i].position);
        }

        planning_scene_interface_ptr->applyCollisionObject(container_object, object_color);

        return container_object.id;
    }

    /**
     * @brief Add floor to the move group interface
     *
     * @param workspace_origin_link link of origin
     * @param planning_scene_interface_ptr planning scene interface nullptr
     * @return the ids of the collision floor
     */
    std::string add_floor(const std::string &workspace_origin_link, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr)
    {
        std_msgs::ColorRGBA object_color;
        object_color.a = 1;
        object_color.r = 0.5;
        object_color.g = 0.5;
        object_color.b = 0.5;

        moveit_msgs::CollisionObject floor;
        floor.id = "floor";
        floor.header.frame_id = workspace_origin_link;
        floor.operation = moveit_msgs::CollisionObject::ADD;

        floor.primitives.resize(1);
        floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

        floor.primitives[0].dimensions.resize(3);

        floor.primitives[0].dimensions[0] = 5;
        floor.primitives[0].dimensions[1] = 5;
        floor.primitives[0].dimensions[2] = 0.01;

        floor.pose.position.z = -0.01;
        floor.pose.orientation.w = 1;

        planning_scene_interface_ptr->applyCollisionObject(floor, object_color);

        return floor.id;
    }

    /**
     * @brief Add floor to the move group interface
     *
     * @param workspace_origin_link link of origin
     * @param planning_scene_interface_ptr planning scene interface nullptr
     * @param x_dimension x dimension
     * @param y_dimension y dimension
     * @param x_pos x position
     * @param y_pos y position
     * @param floor_name floor id
     * @return the ids of the collision floor
     */
    std::string add_floor(const std::string &workspace_origin_link, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr, double x_dimension, double y_dimension, double x_pos, double y_pos, std::string floor_name)
    {
        std_msgs::ColorRGBA object_color;
        object_color.a = 1;
        object_color.r = 0.5;
        object_color.g = 0.5;
        object_color.b = 0.5;

        moveit_msgs::CollisionObject floor;
        floor.id = floor_name;
        floor.header.frame_id = workspace_origin_link;
        floor.operation = moveit_msgs::CollisionObject::ADD;

        floor.primitives.resize(1);
        floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

        floor.primitives[0].dimensions.resize(3);

        floor.primitives[0].dimensions[0] = x_dimension;
        floor.primitives[0].dimensions[1] = y_dimension;
        floor.primitives[0].dimensions[2] = 0.01;

        floor.pose.position.x = x_pos;
        floor.pose.position.y = y_pos;
        floor.pose.position.z = -0.01;
        floor.pose.orientation.w = 1;

        planning_scene_interface_ptr->applyCollisionObject(floor, object_color);

        return floor.id;
    }

    std::string add_blocking(const std::string &workspace_origin_link, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr,
                             double x_dimension, double y_dimension, double z_dimension,
                             double x_position, double y_position, double z_position,
                             std::string block_id)
    {
        std_msgs::ColorRGBA object_color;
        object_color.a = 0.2;
        object_color.r = 1;
        object_color.g = 1;
        object_color.b = 1;

        moveit_msgs::CollisionObject block;

        block.id = block_id;
        block.header.frame_id = workspace_origin_link;
        block.operation = moveit_msgs::CollisionObject::ADD;

        block.primitives.resize(1);
        block.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

        block.primitives[0].dimensions.resize(3);

        block.primitives[0].dimensions[0] = x_dimension;
        block.primitives[0].dimensions[1] = y_dimension;
        block.primitives[0].dimensions[2] = z_dimension;

        block.pose.position.x = x_position;
        block.pose.position.y = y_position;
        block.pose.position.z = z_position;
        block.pose.orientation.w = 1;

        planning_scene_interface_ptr->applyCollisionObject(block, object_color);

        return block.id;
    }

    /**
     * @brief Add the package to the planning scene
     *
     * @param pkg_pose the package pose
     * @param pkg_size the package size
     * @param planning_scene_interface_ptr planning scene interface pointer
     * @param workspace_origin_link the origin link of the workspace
     * @return moveit_msgs::CollisionObject the created package
     */
    moveit_msgs::CollisionObject add_package(geometry_msgs::Pose &pkg_pose, geometry_msgs::Point &pkg_size, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr, const std::string &workspace_origin_link)
    {
        std_msgs::ColorRGBA package_color;
        package_color.a = 1;
        package_color.r = 0.5;
        package_color.g = 0.5;
        package_color.b = 0.5;

        moveit_msgs::CollisionObject package;
        package.id = "package";
        package.header.frame_id = workspace_origin_link;
        package.operation = moveit_msgs::CollisionObject::ADD;

        package.primitives.resize(1);
        package.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        package.primitives[0].dimensions.resize(3);

        package.primitives[0].dimensions[0] = pkg_size.x;
        package.primitives[0].dimensions[1] = pkg_size.y;
        package.primitives[0].dimensions[2] = pkg_size.z;

        package.pose = pkg_pose;
        package.pose.position.z -= pkg_size.z / 2;

        planning_scene_interface_ptr->applyCollisionObject(package, package_color);

        return package;
    }

    /**
     * @brief Attach the package to the move group end effector
     *
     * @param pkg_size the size of the package
     * @param planning_scene_interface_ptr the pointer of the planning scene interface
     * @param move_group_interface_ptr the pointer of the move group interface
     */
    void attach_package(geometry_msgs::Point &pkg_size, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr, moveit::planning_interface::MoveGroupInterface *move_group_interface_ptr)
    {

        moveit_msgs::AttachedCollisionObject attached_package;

        attached_package.link_name = move_group_interface_ptr->getEndEffectorLink();
        attached_package.touch_links = {"iiwa_blue_suction_gripper"};

        attached_package.object.id = "package_attached";
        attached_package.object.header.frame_id = move_group_interface_ptr->getEndEffectorLink();
        attached_package.object.operation = moveit_msgs::CollisionObject::ADD;

        attached_package.object.primitives.resize(1);
        attached_package.object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        attached_package.object.primitives[0].dimensions.resize(3);
        attached_package.object.primitives[0].dimensions[0] = pkg_size.x;
        attached_package.object.primitives[0].dimensions[1] = pkg_size.y;
        attached_package.object.primitives[0].dimensions[2] = pkg_size.z;

        geometry_msgs::Pose attached_pose;
        attached_pose.position.z = pkg_size.z / 2;
        attached_pose.orientation.w = 1;

        attached_package.object.pose = attached_pose;

        planning_scene_interface_ptr->applyAttachedCollisionObject(attached_package);
    }

    /**
     * @brief Remove the package in the planning scene
     *
     * @param planning_scene_interface_ptr the pointer of the planning scene interface
     */
    void detach_package(moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr)
    {
        moveit_msgs::AttachedCollisionObject attached_package;

        attached_package.object.id = "package_attached";
        attached_package.object.operation = moveit_msgs::CollisionObject::REMOVE;

        planning_scene_interface_ptr->applyAttachedCollisionObject(attached_package);
        planning_scene_interface_ptr->applyCollisionObject(attached_package.object);
    }
}