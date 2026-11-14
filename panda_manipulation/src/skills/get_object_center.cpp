#include <thread>
#include <utility> 
#include "panda_manipulation/skills/get_object_center.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;
using GetObjectCenterStatus = PerceptionExecutor::GetObjectCenterStatus;

// 初始化函数
GetObjectCenterServer::GetObjectCenterServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<PerceptionExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
     server_ = rclcpp_action::create_server<GetObjectCenter>(
        node_, "/manipulator/get_object_center",
        std::bind(&GetObjectCenterServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GetObjectCenterServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&GetObjectCenterServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse GetObjectCenterServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetObjectCenter::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse GetObjectCenterServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 
void GetObjectCenterServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

// 
void GetObjectCenterServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<GetObjectCenter::Result>();
    auto feedback = std::make_shared<GetObjectCenter::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "perceiving";
    goal_handle->publish_feedback(feedback);

    // 调用执行器获取物体的中心位置
    geometry_msgs::msg::PoseStamped object_pose;
    GetObjectCenterStatus task_result = executor_->getObjectCenterPositionOnce(object_pose);
            
    // 根据返回的状态向客户端返回结果
    auto transferErrorCode = [&task_result]() {
        switch (task_result) {
            case GetObjectCenterStatus::SUCCESS:
                return ErrorCode::SUCCESS;

            case GetObjectCenterStatus::NO_DATA:
                return ErrorCode::PERCEPTION_NO_DATA;

            case GetObjectCenterStatus::NO_OBJECT:
                return ErrorCode::PERCEPTION_NO_OBJECT;

            default:
                return ErrorCode::SKILL_FAILED;
        }
    };

    result->object_pose = object_pose;
    result->error_code = transferErrorCode();
    result->success = (result->error_code == ErrorCode::SUCCESS);
    result->message = (result->error_code == ErrorCode::SUCCESS) ? "Get Object Center Success!" : "Get Object Center Failed!";

    if (result->success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}