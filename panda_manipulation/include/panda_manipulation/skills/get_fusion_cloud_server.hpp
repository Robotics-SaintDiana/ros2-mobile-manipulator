#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/get_fusion_cloud.hpp>
#include "panda_manipulation/task_executor.hpp"

class GetFusionCloudServer
{
    public:
        GetFusionCloudServer(rclcpp::Node::SharedPtr node, std::shared_ptr<TaskExecutor> executor);

    private:
        using GetFusionCloud = robot_manipulation_msgs::action::GetFusionCloud;
        using GoalHandle = rclcpp_action::ServerGoalHandle<GetFusionCloud>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<GetFusionCloud>::SharedPtr server_;
        std::shared_ptr<TaskExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetFusionCloud::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};