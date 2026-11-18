#include "robot_bt_manager/nodes/execute_grasp.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会从黑板中读入筛选后的抓取位姿
BT::PortsList ExecuteGraspNode::providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("filter_poses"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取筛选后的抓取位姿,取第一个位姿进行抓取
bool ExecuteGraspNode::prepareGoal(Action::Goal& goal) {
    // 读取数据
    auto filter_poses = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("filter_poses");

    // 读取不到数据报错
    if (!filter_poses) return false;

    // 读取到则准备发给服务端的数据
    auto result = *filter_poses;
    goal.pick_pose = result[0];
    return true;
}

// 该节点需要向黑板写入任务的错误码
BT::NodeStatus ExecuteGraspNode::onResult(const WrappedResult& wrapped) {
     // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}