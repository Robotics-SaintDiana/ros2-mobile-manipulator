#pragma once

#include <memory>
#include <cmath>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "robot_bt_manager/ros_action_node.hpp"

class NavigateToPoseNode : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
    public:
        NavigateToPoseNode(const std::string& name,
                            const BT::NodeConfiguration& config,
                            rclcpp::Node::SharedPtr node)
            : RosActionNode(name, config, node, "/navigate_to_pose") {}
        
        // Nav2 需要从黑板中获取导航的目标位置
        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<geometry_msgs::msg::PoseStamped>("nav2_position"),
            };
        }

    private:
        bool prepareGoal(Action::Goal& goal) override {
            // 从黑板中读取导航的目标位置，读取不到则报错
            auto nav2_position = getInput<geometry_msgs::msg::PoseStamped>("nav2_position");
            if (!nav2_position) return false;

            // 组装给服务端发送的任务目标
            goal.pose = *nav2_position;
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = node_->now();
            
            return true;
        }

        BT::NodeStatus onResult(const WrappedResult& wrapped) override {
            (void)wrapped;
            return BT::NodeStatus::SUCCESS;
        }
};