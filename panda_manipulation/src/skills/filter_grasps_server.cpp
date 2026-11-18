#include <thread>
#include <utility> 
#include "panda_manipulation/skills/filter_grasps_server.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 初始化函数
FilterGraspsServer::FilterGraspsServer(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<GraspFeasibilityFilter> executor)
    : node_(std::move(node)), executor_(std::move(executor))
{
     server_ = rclcpp_action::create_server<FilterGrasps>(
        node_, "/manipulator/filter_grasps",
        std::bind(&FilterGraspsServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&FilterGraspsServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&FilterGraspsServer::handle_accepted, this, std::placeholders::_1)
    );
}

// 目标接收回调
rclcpp_action::GoalResponse FilterGraspsServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const FilterGrasps::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消请求回调
rclcpp_action::CancelResponse FilterGraspsServer::handle_cancel(std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 目标被接受后的回调
void FilterGraspsServer::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    // 另外开一个线程执行机械臂的运动，防止堵塞主线程
    std::thread([this, goal_handle]() { 
         execute(goal_handle); 
    }).detach();
}

void FilterGraspsServer::execute(std::shared_ptr<GoalHandle> goal_handle) {
    // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
    auto result = std::make_shared<FilterGrasps::Result>();
    auto feedback = std::make_shared<FilterGrasps::Feedback>();

    // 任务处于规划阶段，反馈给客户端
    feedback->stage = "filtering";
    goal_handle->publish_feedback(feedback);

    // 获取 GraspNet 所有输出的评估结果，以及机械臂当前的关节角度值,以及物体的中心位置
    const auto goal = goal_handle->get_goal();
    auto candidates = goal->candidates;
    auto scores = goal->scores;
    auto widths = goal->widths;
    auto joint_values = goal->joint_values;
    double center_x = goal->object_pose.pose.position.x;
    double center_y = goal->object_pose.pose.position.y;

    std::vector<double> scores_double(goal->scores.begin(), goal->scores.end());
    std::vector<double> widths_double(goal->widths.begin(), goal->widths.end());

    // 调用执行器获取筛选后的合适抓取位姿
    auto filterResult = executor_->graspPoseFilter(
        candidates, scores_double, widths_double, joint_values, center_x, center_y);

    // 根据返回的状态向客户端返回结果
    if (!filterResult.empty()) {
        result->success = true;
        result->error_code = ErrorCode::SUCCESS;
        result->message = "Filter Grasps Success";

        const size_t array_len = filterResult.size();
        result->filter_poses.resize(array_len);
        result->filter_scores.resize(array_len);
        for (size_t i = 0; i < array_len; i++) {
            result->filter_poses[i] = filterResult[i].pose;
            result->filter_scores[i] = filterResult[i].score;
        }

        goal_handle->succeed(result);
    } else {
        result->success = false;
        result->error_code = ErrorCode::FILTER_NO_FEASIBLE;
        result->message = "Filter Grasps Failed";
        goal_handle->abort(result);
    }
}