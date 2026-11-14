#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/get_object_center.hpp>
#include "panda_manipulation/perception_executor.hpp"

class GetObjectCenterServer
{
    public:
        GetObjectCenterServer(rclcpp::Node::SharedPtr node, std::shared_ptr<PerceptionExecutor> executor);

    private:
        using GetObjectCenter = robot_manipulation_msgs::action::GetObjectCenter;
        using GoalHandle = rclcpp_action::ServerGoalHandle<GetObjectCenter>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<GetObjectCenter>::SharedPtr server_;
        std::shared_ptr<PerceptionExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetObjectCenter::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};