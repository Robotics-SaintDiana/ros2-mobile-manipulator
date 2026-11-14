#include "robot_bt_manager/nodes/move_home.hpp"

// 初始化函数 --- 初始化基类与客户端
MoveHomeNode::MoveHomeNode(const std::string& name, const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) 
{
    client_ = rclcpp_action::create_client<MoveHome>(node_, "/manipulator/move_home");
}

// 告知行为树有哪些输入输出端口
// 这里直接返回，表示不需要从行为树黑板读取参数，也不需要把结果写回黑板
BT::PortsList MoveHomeNode::providedPorts() {
    return {};
}

// 行为树第一次 tick() 到这个节点时调用此函数
// 主要内容重置所有变量可能残留的状态
BT::NodeStatus MoveHomeNode::onStart() {
    is_goal_sent_ = false;
    start_time_ = std::chrono::steady_clock::now();

    active_handle_.reset();
    goal_future_ = {};
    result_future_ = {};

    return BT::NodeStatus::RUNNING;
}

// 轮询状态的核心函数，行为树通过不断调用它来感知任务进度
BT::NodeStatus MoveHomeNode::onRunning() {
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
        goal_future_ = client_->async_send_goal(MoveHome::Goal{});
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

    // 需要判断服务端中所执行任务是否成功
    return wrapped.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// 取消这个节点时，发送取消请求，清理句柄和 future
void MoveHomeNode::onHalted() {
    // 1. 发送取消请求
    if (active_handle_) {
        client_->async_cancel_goal(active_handle_);
        active_handle_ = nullptr;
    }
    
    // 2. 清理句柄和 future
    goal_future_ = {};
    result_future_ = {};
}