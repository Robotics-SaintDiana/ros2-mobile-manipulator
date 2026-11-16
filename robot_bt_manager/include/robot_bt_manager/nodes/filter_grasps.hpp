#pragma once

#include <memory>
#include <behaviortree_cpp_v3/action_node.h>
#include "robot_bt_manager/ros_action_node.hpp"
#include <robot_manipulation_msgs/action/filter_grasps.hpp>

class FilterGraspsNode : public RosActionNode<robot_manipulation_msgs::action::FilterGrasps> 
{
    public:
        FilterGraspsNode(const std::string& name,
                            const BT::NodeConfiguration& config,
                            rclcpp::Node::SharedPtr node)
            : RosActionNode(name, config, node, "/filter/filter_grasps") {}

        static BT::PortsList providedPorts();

    private:
        bool prepareGoal(Action::Goal& goal) override;
        BT::NodeStatus onResult(const WrappedResult& wrapped) override;
};