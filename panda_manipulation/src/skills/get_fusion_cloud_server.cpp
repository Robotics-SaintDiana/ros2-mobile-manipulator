#include <thread>
#include <utility> 
#include "panda_manipulation/skills/get_fusion_cloud_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;
using GetFusionCloudStatus = TaskExecutor::GetFusionCloudStatus;

// 初始化函数
GetFusionCloudServer::GetFusionCloudServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<TaskExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
     server_ = rclcpp_action::create_server<GetFusionCloud>(
        node_, "/manipulator/get_fusion_cloud",
        std::bind(&GetFusionCloudServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GetFusionCloudServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&GetFusionCloudServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse GetFusionCloudServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetFusionCloud::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse GetFusionCloudServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void GetFusionCloudServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void GetFusionCloudServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<GetFusionCloud::Result>();
    auto feedback = std::make_shared<GetFusionCloud::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "fusing";
    goal_handle->publish_feedback(feedback);

    // 调用执行器获取融合后的点云数据
    const auto goal = goal_handle->get_goal();
    auto object_pose = goal->object_pose;
    sensor_msgs::msg::PointCloud2 fusion_cloud;
    GetFusionCloudStatus task_result = executor_->captureFusionCloud(object_pose, fusion_cloud);

    // 根据返回的状态向客户端返回结果
    auto transferErrorCode = [&task_result]() {
        switch (task_result) {
            case GetFusionCloudStatus::SUCCESS:
                return ErrorCode::SUCCESS;

            case GetFusionCloudStatus::GET_TF_FAILED:
                return ErrorCode::SKILL_FAILED;

            case GetFusionCloudStatus::FUSION_FAILED:
                return ErrorCode::PERCEPTION_FUSION_FAILED;

            default:
                return ErrorCode::SKILL_FAILED;
        }
    };

    result->fusion_cloud = fusion_cloud;
    result->error_code = transferErrorCode();
    result->success = (result->error_code == ErrorCode::SUCCESS);
    result->message = (result->error_code == ErrorCode::SUCCESS) ? "Get Fusion Cloud Success!" : "Get Fusion Cloud Failed!";

    if (result->success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}