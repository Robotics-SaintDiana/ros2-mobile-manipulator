#include <thread>
#include <utility> 
#include "panda_manipulation/skills/execute_place_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;
using placeExecutorStatus = ManipulatorExecutor::placeExecutorStatus;

// 初始化函数
ExecutePlaceServer::ExecutePlaceServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
     server_ = rclcpp_action::create_server<ExecutePlace>(
        node_, "/manipulator/execute_place",
        std::bind(&ExecutePlaceServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ExecutePlaceServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ExecutePlaceServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse ExecutePlaceServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecutePlace::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse ExecutePlaceServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void ExecutePlaceServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void ExecutePlaceServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<ExecutePlace::Result>();
    auto feedback = std::make_shared<ExecutePlace::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "executing";
    goal_handle->publish_feedback(feedback);

    // 获取放置位姿
    const auto goal = goal_handle->get_goal();
    auto place_pose = goal->place_pose;

    // 调用执行器进行放置
    placeExecutorStatus task_result = executor_->placeExecutor(place_pose);

     // 根据返回的状态向客户端返回结果
    auto transferErrorCode = [&task_result]() {
        switch (task_result) {
            case placeExecutorStatus::SUCCESS:
                return ErrorCode::SUCCESS;
            
            case placeExecutorStatus::OBJECT_FALL:
                return ErrorCode::PLACE_OBJECT_FALL;

            case placeExecutorStatus::NOT_IN_PLATE:
                return ErrorCode::PLACE_NOT_IN_PLATE;
            
            default:
                return ErrorCode::SKILL_FAILED;
        }
    };

    result->error_code = transferErrorCode();
    result->success = (result->error_code == ErrorCode::SUCCESS);
    result->message = (result->error_code == ErrorCode::SUCCESS) ? "Place Executor Success!" : "Place Executor Failed!";

    if (result->success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}