#include <thread>
#include <utility> 
#include "panda_manipulation/skills/get_joint_values_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 初始化函数
GetJointValuesServer::GetJointValuesServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ManipulatorExecutor> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
    // 初始化该服务端
    server_ = rclcpp_action::create_server<GetJointValues>(
        node_, "/manipulator/get_joint_values",
        std::bind(&GetJointValuesServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GetJointValuesServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&GetJointValuesServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse GetJointValuesServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GetJointValues::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse GetJointValuesServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void GetJointValuesServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
        execute(goal_handle); 
    }).detach();       
}

void GetJointValuesServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<GetJointValues::Result>();
    auto feedback = std::make_shared<GetJointValues::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "getting";
    goal_handle->publish_feedback(feedback);

    // 调用执行器获取机械臂当前的关节值
    auto joint_values_opt = executor_->getCurrentJointValues();

    // 读取成功或者失败后的处理
    if (!joint_values_opt) {
        result->success = false;
        result->error_code = ErrorCode::SKILL_FAILED;
        result->message = "Get Joint Values Failed";
        result->joint_values = {};
        goal_handle->abort(result);
    } else {
        const std::vector<double>& joint_values = joint_values_opt.value();
        result->success = true;
        result->joint_values = joint_values;
        result->message = "Get Joint Values Success";
        result->error_code = ErrorCode::SUCCESS;
        goal_handle->succeed(result);
    }
}