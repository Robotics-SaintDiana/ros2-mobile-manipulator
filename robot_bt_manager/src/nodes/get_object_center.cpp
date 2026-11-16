#include "robot_bt_manager/nodes/get_object_center.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会向黑板中写入物体的位姿数据及任务的错误码
BT::PortsList GetObjectCenterNode::providedPorts() {
    return {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("object_pose"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时不需要向黑板中读取任何数据
bool GetObjectCenterNode::prepareGoal(Action::Goal& goal) {
    (void)goal;
    return true;
}

// 拿到服务端返回的数据后，该节点需要向黑板写入物体的中心位置数据
// 此外该节点也需要向黑板写入错误码，以便 BT 根据错误码进行重试等功能
BT::NodeStatus GetObjectCenterNode::onResult(const WrappedResult& wrapped) {
    // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入物体的中心位置数据
    setOutput("object_pose", wrapped.result->object_pose);
    return BT::NodeStatus::SUCCESS;
}