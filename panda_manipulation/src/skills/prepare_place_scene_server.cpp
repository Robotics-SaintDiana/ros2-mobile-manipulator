#include <thread>
#include <utility> 
#include "panda_manipulation/skills/prepare_place_scene_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 初始化函数
PreparePlaceSceneServer::PreparePlaceSceneServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
    server_ = rclcpp_action::create_server<PreparePlaceScene>(
        node_, "/manipulator/prepare_place_scene",
        std::bind(&PreparePlaceSceneServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PreparePlaceSceneServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PreparePlaceSceneServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse PreparePlaceSceneServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PreparePlaceScene::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse PreparePlaceSceneServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void PreparePlaceSceneServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void PreparePlaceSceneServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<PreparePlaceScene::Result>();
    auto feedback = std::make_shared<PreparePlaceScene::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "preparing";
    goal_handle->publish_feedback(feedback);

    // 获取 Action 的 Goal，从中获取托盘的中心位置作为物体添加的地方
    const auto goal = goal_handle->get_goal();
    const double center_x = goal->plate_pose.pose.position.x;
    const double center_y = goal->plate_pose.pose.position.y;
    const double center_z = goal->plate_pose.pose.position.z;

    // 向 Moveit 的规划场景中添加物体
    bool is_success = executor_->preparePlaceCollisionScene(center_x, center_y, center_z);

    // 根据添加结果向客户端返回结果
    result->success = is_success;
    result->error_code = is_success ? ErrorCode::SUCCESS : ErrorCode::SKILL_FAILED;
    result->message = is_success ? "Prepare Place Scene Success!" : "Prepare Place Scene Failed!";

    if (is_success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}