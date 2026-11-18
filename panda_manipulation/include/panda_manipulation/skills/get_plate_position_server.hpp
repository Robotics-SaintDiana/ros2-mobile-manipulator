#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/get_plate_position.hpp>
#include "panda_manipulation/perception_executor.hpp"

class GetPlatePositionServer
{
    public:
        GetPlatePositionServer(rclcpp::Node::SharedPtr node, std::shared_ptr<PerceptionExecutor> executor);

    private:
        using GetPlatePosition = robot_manipulation_msgs::action::GetPlatePosition;
        using GoalHandle = rclcpp_action::ServerGoalHandle<GetPlatePosition>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<GetPlatePosition>::SharedPtr server_;
        std::shared_ptr<PerceptionExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetPlatePosition::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};