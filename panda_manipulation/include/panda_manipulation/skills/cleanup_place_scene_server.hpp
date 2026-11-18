#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/cleanup_place_scene.hpp>
#include "panda_manipulation/manipulator_executor.hpp"

class CleanUpPlaceSceneServer
{
    public:
        CleanUpPlaceSceneServer(rclcpp::Node::SharedPtr node, std::shared_ptr<ManipulatorExecutor> executor);

    private:
        using CleanUpPlaceScene = robot_manipulation_msgs::action::CleanupPlaceScene;
        using GoalHandle = rclcpp_action::ServerGoalHandle<CleanUpPlaceScene>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<CleanUpPlaceScene>::SharedPtr server_;
        std::shared_ptr<ManipulatorExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const CleanUpPlaceScene::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);

};