#include "robot_bt_manager/nodes/get_joint_values.hpp"

// 该节点会向黑板中写入获得的机械臂关节角度
BT::PortsList GetJointValuesNode::providedPorts() {
    return {
        BT::OutputPort<std::vector<double>>("joint_values"),
    };
}

// 该节点向服务端发送任务目标时不需要向黑板中读取任何数据
bool GetJointValuesNode::prepareGoal(Action::Goal& goal) {
    (void)goal;
    return true;
}

// 该节点需要向黑板写入获得的机械臂关节角度
BT::NodeStatus GetJointValuesNode::onResult(const WrappedResult& wrapped) {
    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入获得的机械臂关节角度
    setOutput("joint_values", wrapped.result->joint_values);
    return BT::NodeStatus::SUCCESS;
}