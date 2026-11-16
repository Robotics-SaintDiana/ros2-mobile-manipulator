#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/filter_grasps.hpp>
#include "panda_manipulation/grasp_feasibility_filter.hpp"

class FilterGraspsServer
{
    public:
        FilterGraspsServer(rclcpp::Node::SharedPtr node, std::shared_ptr<GraspFeasibilityFilter> executor);
    
    private:
        using FilterGrasps = robot_manipulation_msgs::action::FilterGrasps;
        using GoalHandle = rclcpp_action::ServerGoalHandle<FilterGrasps>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<FilterGrasps>::SharedPtr server_;
        std::shared_ptr<GraspFeasibilityFilter> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const FilterGrasps::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};