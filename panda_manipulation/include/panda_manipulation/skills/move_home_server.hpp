#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/move_home.hpp>
#include "panda_manipulation/manipulator_executor.hpp"

class MoveHomeServer
{
    public:
        MoveHomeServer(rclcpp::Node::SharedPtr node, std::shared_ptr<ManipulatorExecutor> executor);

    private:
        using MoveHome = robot_manipulation_msgs::action::MoveHome;
        using GoalHandle = rclcpp_action::ServerGoalHandle<MoveHome>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<MoveHome>::SharedPtr server_;
        std::shared_ptr<ManipulatorExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveHome::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};