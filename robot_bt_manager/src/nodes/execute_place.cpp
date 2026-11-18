#include "robot_bt_manager/nodes/execute_place.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会从黑板中读入放置位姿
BT::PortsList ExecutePlaceNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读入放置位姿
bool ExecutePlaceNode::prepareGoal(Action::Goal& goal) {
    // 读取数据
    auto place_pose = getInput<geometry_msgs::msg::PoseStamped>("place_pose");

    // 读取不到数据报错
    if (!place_pose) return false;

    // 读取到则准备发给服务端的数据
    goal.place_pose = *place_pose;
    return true;
}

// 该节点需要向黑板写入任务的错误码
BT::NodeStatus ExecutePlaceNode::onResult(const WrappedResult& wrapped) {
     // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}