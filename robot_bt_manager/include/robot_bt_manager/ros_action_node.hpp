#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

template <typename ActionT>
class RosActionNode : public BT::StatefulActionNode
{
    public:
        using Action = ActionT;
        using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
        using WrappedResult = typename GoalHandle::WrappedResult;

        RosActionNode(const std::string& name,
                    const BT::NodeConfiguration& config,
                    rclcpp::Node::SharedPtr node,
                    const std::string& action_name)
            : BT::StatefulActionNode(name, config), node_(std::move(node))
        {
            client_ = rclcpp_action::create_client<Action>(node_, action_name);
        }

        // ---------- 子类必须实现的函数 ----------
        // 1. 填充向服务端发送的任务目标，有些子类需要从黑板中读取数据，有些无需
        virtual bool prepareGoal(typename Action::Goal& goal) = 0;

        // 2. 处理服务端返回的结果，有些子类接收到结果后还需要向黑板中写入数据，有些无需
        virtual BT::NodeStatus onResult(const WrappedResult& wrapped) = 0;

        // ---------- 基类中统一实现的函数 ----------
        // 行为树第一次 tick() 到这个节点时调用此函数
        // 主要内容重置所有变量可能残留的状态(防止重试机制再次调用)，以及准备向服务端发送的任务目标
        BT::NodeStatus onStart() override {
            // 1. 重置所有变量状态
            is_goal_sent_ = false;
            active_handle_.reset();
            goal_future_ = {};
            result_future_ = {};
            start_time_ = std::chrono::steady_clock::now();

            // 2. 准备向服务端发送的任务目标
            goal_ = typename Action::Goal{};
            if (!prepareGoal(goal_)) return BT::NodeStatus::FAILURE;

            return BT::NodeStatus::RUNNING;
        }

        // 轮询状态的核心函数，行为树通过不断调用它来感知任务进度
        BT::NodeStatus onRunning() override {
            // 1. 首先需要确定服务器是否在线，若 5s 内没有响应判定失败
            if (!is_goal_sent_) {
                if (!client_->action_server_is_ready()) {
                    // 服务器未响应则计算到目前为止过去了多长时间
                    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
                    if (elapsed >= std::chrono::seconds(5)) {
                        return BT::NodeStatus::FAILURE;
                    }

                    return BT::NodeStatus::RUNNING;
                }

                // 如果服务器已经在线，向服务器发送任务目标
                goal_future_ = client_->async_send_goal(goal_);
                is_goal_sent_ = true;
                return BT::NodeStatus::RUNNING;
            }

            // 2. 首先需要确定服务端是否接收了任务
            // 2.1 等待服务端返回的结果
            if (!goal_future_.valid() || goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }

            // 2.2 判断服务端是否接收了任务
            active_handle_ = goal_future_.get();
            if (!active_handle_) {
                return BT::NodeStatus::FAILURE;
            }

            // 3. 等待任务完成的最终结果
            if (!result_future_.valid()) {
                result_future_ = client_->async_get_result(active_handle_);
            }

            if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }

            // 4. 处理任务返回的结果
            auto wrapped = result_future_.get();
            if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
                return BT::NodeStatus::FAILURE;
            }

            return onResult(wrapped);
        }

        // 当行为树决定取消这个节点时调用
        void onHalted() override {
            // 1. 发送取消请求
            if (active_handle_) {
                client_->async_cancel_goal(active_handle_);
                active_handle_ = nullptr;
            }
            
            // 2. 清理句柄和 future
            goal_future_ = {};
            result_future_ = {};
        }
    
    protected:
        // 外部传入的 ROS2 节点
        rclcpp::Node::SharedPtr node_;

    private:
        // 外部传入的客户端
        typename rclcpp_action::Client<Action>::SharedPtr client_;

        // 是否向服务端发送了任务目标，以及发送时的时间
        bool is_goal_sent_ = false;
        std::chrono::steady_clock::time_point start_time_;

        // 向服务端发送的任务目标
        typename Action::Goal goal_;

        // 当前正在执行的 Goal 句柄，Goal 被接受后赋值，用于后续 cancel 或查询状态
        typename GoalHandle::SharedPtr active_handle_;

        // 发送 Goal 后返回的 future，以及等待最终结果的 future
        std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
        std::shared_future<typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult> result_future_;
};