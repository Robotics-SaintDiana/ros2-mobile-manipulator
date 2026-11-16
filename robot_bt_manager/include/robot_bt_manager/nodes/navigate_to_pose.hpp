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
        
        // Nav2 需要从黑板中获取目标位置的坐标及朝向
        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("x"),
                BT::InputPort<double>("y"),
                BT::InputPort<double>("yaw"),
            };
        }

    private:
        bool prepareGoal(Action::Goal& goal) override {
            // 从黑板中读取目标位置的坐标和朝向，读取不到则报错
            auto x = getInput<double>("x");
            auto y = getInput<double>("y");
            auto yaw = getInput<double>("yaw");
            if (!x || !y || !yaw) return false;

            // 组装给服务端发送的任务目标
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = node_->now();

            goal.pose.pose.position.x = *x;
            goal.pose.pose.position.y = *y;
            goal.pose.pose.position.z = 0;
            goal.pose.pose.orientation.x = 0.0;
            goal.pose.pose.orientation.y = 0.0;
            goal.pose.pose.orientation.z = std::sin(*yaw / 2.0);
            goal.pose.pose.orientation.w = std::cos(*yaw / 2.0);

            return true;
        }

        BT::NodeStatus onResult(const WrappedResult& wrapped) override {
            (void)wrapped;
            return BT::NodeStatus::SUCCESS;
        }
};