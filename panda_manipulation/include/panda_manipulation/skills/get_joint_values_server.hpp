#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/get_joint_values.hpp>
#include "panda_manipulation/manipulator_executor.hpp"

class GetJointValuesServer
{
    public:
        GetJointValuesServer(rclcpp::Node::SharedPtr node, std::shared_ptr<ManipulatorExecutor> executor);

    private:
        using GetJointValues = robot_manipulation_msgs::action::GetJointValues;
        using GoalHandle = rclcpp_action::ServerGoalHandle<GetJointValues>;

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<GetJointValues>::SharedPtr server_;
        std::shared_ptr<ManipulatorExecutor> executor_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetJointValues::Goal>);
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
        void execute(std::shared_ptr<GoalHandle> goal_handle);
};