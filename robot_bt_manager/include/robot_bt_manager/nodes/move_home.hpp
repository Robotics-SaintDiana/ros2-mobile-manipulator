#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_manipulation_msgs/action/move_home.hpp>

class MoveHomeNode : public BT::StatefulActionNode
{
    public:
        // 初始化函数
        // name --- 行为树节点的名称(与 XML 文件中对应)； config --- 节点配置; node --- ROS2 节点
        MoveHomeNode(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node);

        // 告知行为树有哪些输入输出端口
        static BT::PortsList providedPorts();

        // 行为树第一次 tick() 到这个节点时调用(全生命周期中只调用一次)
        BT::NodeStatus onStart() override;

        // 只要 onStart() 返回了 RUNNING，之后每次行为树 tick() 到这个节点都会调用该函数
        // 轮询状态的核心函数，行为树通过不断调用它来感知任务进度
        BT::NodeStatus onRunning() override;

        // 当行为树决定取消这个节点时调用
        void onHalted() override;

    private:
        using MoveHome = robot_manipulation_msgs::action::MoveHome;
        using GoalHandle = rclcpp_action::ClientGoalHandle<MoveHome>;

        // 外部传入的 ROS2 节点与客户端
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<MoveHome>::SharedPtr client_;

        // 是否向服务端发送了任务目标，以及发送时的时间
        bool is_goal_sent_ = false;
        std::chrono::steady_clock::time_point start_time_;

        // 当前正在执行的 Goal 句柄，Goal 被接受后赋值，用于后续 cancel 或查询状态
        GoalHandle::SharedPtr active_handle_;

        // 发送 Goal 后返回的 future，以及等待最终结果的 future
        std::shared_future<GoalHandle::SharedPtr> goal_future_;
        std::shared_future<rclcpp_action::ClientGoalHandle<MoveHome>::WrappedResult> result_future_;
};