#include <thread>
#include <utility> 
#include "panda_manipulation/skills/execute_grasp_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;
using pickExecutorStatus = ManipulatorExecutor::pickExecutorStatus;

// 初始化函数
ExecuteGraspServer::ExecuteGraspServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
     server_ = rclcpp_action::create_server<ExecuteGrasp>(
        node_, "/manipulator/execute_grasp",
        std::bind(&ExecuteGraspServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ExecuteGraspServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ExecuteGraspServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse ExecuteGraspServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteGrasp::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse ExecuteGraspServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void ExecuteGraspServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void ExecuteGraspServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<ExecuteGrasp::Result>();
    auto feedback = std::make_shared<ExecuteGrasp::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "executing";
    goal_handle->publish_feedback(feedback);

    // 获取筛选后的抓取位姿
    const auto goal = goal_handle->get_goal();
    auto pick_pose = goal->pick_pose;

    // 调用执行器进行抓取
    pickExecutorStatus task_result = executor_->pickExecutor(pick_pose);

    // 根据返回的状态向客户端返回结果
    auto transferErrorCode = [&task_result]() {
        switch (task_result) {
            case pickExecutorStatus::SUCCESS:
                return ErrorCode::SUCCESS;
            
            case pickExecutorStatus::NO_OBJECT:
                return ErrorCode::PICK_NO_OBJECT;

            case pickExecutorStatus::LIFT_FAILED:
                return ErrorCode::PICK_LIFT_FAILED;
            
            default:
                return ErrorCode::SKILL_FAILED;
        }
    };

    result->error_code = transferErrorCode();
    result->success = (result->error_code == ErrorCode::SUCCESS);
    result->message = (result->error_code == ErrorCode::SUCCESS) ? "Pick Executor Success!" : "Pick Executor Failed!";

    if (result->success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}