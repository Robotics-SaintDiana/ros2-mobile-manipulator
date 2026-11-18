#pragma once

#include <memory>
#include <behaviortree_cpp_v3/action_node.h>
#include "robot_bt_manager/ros_action_node.hpp"
#include <robot_manipulation_msgs/action/cleanup_place_scene.hpp>

class CleanUpPlaceSceneNode : public RosActionNode<robot_manipulation_msgs::action::CleanupPlaceScene> 
{
    public:
        CleanUpPlaceSceneNode(const std::string& name,
                            const BT::NodeConfiguration& config,
                            rclcpp::Node::SharedPtr node)
            : RosActionNode(name, config, node, "/manipulator/cleanup_place_scene") {}

        static BT::PortsList providedPorts();

    private:
        bool prepareGoal(Action::Goal& goal) override;
        BT::NodeStatus onResult(const WrappedResult& wrapped) override;
};