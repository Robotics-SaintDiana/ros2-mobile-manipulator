#include <thread>
#include <utility> 
#include "panda_manipulation/skills/move_home_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 初始化函数
MoveHomeServer::MoveHomeServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
    // 初始化该服务端
    server_ = rclcpp_action::create_server<MoveHome>(
        node_, "/manipulator/move_home",
        std::bind(&MoveHomeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveHomeServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveHomeServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse MoveHomeServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveHome::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse MoveHomeServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void MoveHomeServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
        execute(goal_handle); 
    }).detach();       
}

void MoveHomeServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<MoveHome::Result>();
    auto feedback = std::make_shared<MoveHome::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "planning";
    goal_handle->publish_feedback(feedback);

    // 调用执行器控制机械臂运动
    bool is_success = executor_->moveHome();

    // 根据机械臂运动的结果向客户端返回结果
    result->success = is_success;
    result->error_code = is_success ? ErrorCode::SUCCESS : ErrorCode::SKILL_FAILED;
    result->message = is_success ? "Move Home Success!" : "Move Home Failed!";

    if (is_success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}