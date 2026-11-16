#include "robot_bt_manager/nodes/prepare_pick_scene.hpp"

// 该节点会从黑板中读取物体的中心位置
BT::PortsList PreparePickSceneNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取物体的中心位置坐标
bool PreparePickSceneNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中读取物体的中心位置坐标
    auto object_pose = getInput<geometry_msgs::msg::PoseStamped>("object_pose");

    // 读取不到数据报错，读取到则准备发给服务端的数据
    if (!object_pose) return false;

    goal.object_pose = *object_pose;
    return true;
}

// 拿到服务端返回的数据后仅需要检查任务是否成功
BT::NodeStatus PreparePickSceneNode::onResult(const WrappedResult& wrapped) {
    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

