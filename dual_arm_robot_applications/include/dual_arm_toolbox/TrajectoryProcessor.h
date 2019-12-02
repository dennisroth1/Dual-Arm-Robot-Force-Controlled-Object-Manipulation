//
// 
//

#ifndef PROJECT_TRAJECTORYPROCESSOR_H
#define PROJECT_TRAJECTORYPROCESSOR_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <controller_manager/controller_manager.h>
//#include <moveit_ros_control_interface>

// Rviz
#include <moveit_msgs/DisplayTrajectory.h>

// Trajectory tools
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// KDL
#include <kdl/frames_io.hpp>


namespace lcr_toolbox {
    class TrajectoryProcessor {
    protected:
    public:

        void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed); 

    };
}//namespace
#endif //PROJECT_TRAJECTORYPROCESSOR_H
