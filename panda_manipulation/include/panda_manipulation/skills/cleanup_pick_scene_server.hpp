#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/cleanup_pick_scene.hpp>
#include "panda_manipulation/manipulator_executor.hpp"

class CleanUpPickSceneServer
{
    public:
        CleanUpPickSceneServer(rclcpp::Node::SharedPtr node, std::shared_ptr<ManipulatorExecutor> executor);

    private:
        using CleanUpPickScene = robot_manipulation_msgs::action::CleanupPickScene;
        using GoalHandle = rclcpp_action::ServerGoalHandle<CleanUpPickScene>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<CleanUpPickScene>::SharedPtr server_;
        std::shared_ptr<ManipulatorExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const CleanUpPickScene::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};