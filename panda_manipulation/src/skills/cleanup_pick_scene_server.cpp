#include <thread>
#include <utility> 
#include "panda_manipulation/skills/cleanup_pick_scene_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 初始化函数
CleanUpPickSceneServer::CleanUpPickSceneServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
    server_ = rclcpp_action::create_server<CleanUpPickScene>(
        node_, "/manipulator/cleanup_pick_scene",
        std::bind(&CleanUpPickSceneServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CleanUpPickSceneServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&CleanUpPickSceneServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse CleanUpPickSceneServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const CleanUpPickScene::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse CleanUpPickSceneServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void CleanUpPickSceneServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void CleanUpPickSceneServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<CleanUpPickScene::Result>();
    auto feedback = std::make_shared<CleanUpPickScene::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "removing";
    goal_handle->publish_feedback(feedback);

    // 向 Moveit 的规划场景中删除物体
    bool is_success = executor_->removePickCollisionScene();

    // 根据添加结果向客户端返回结果
    result->success = is_success;
    result->error_code = is_success ? ErrorCode::SUCCESS : ErrorCode::SKILL_FAILED;
    result->message = is_success ? "CleanUp Pick Scene Success!" : "CleanUp Pick Scene Failed!";

    if (is_success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}