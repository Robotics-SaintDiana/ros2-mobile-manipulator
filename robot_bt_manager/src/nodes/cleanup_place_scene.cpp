#include "robot_bt_manager/nodes/cleanup_place_scene.hpp"

// 这里直接返回，表示不需要从行为树黑板读取参数，也不需要把结果写回黑板
BT::PortsList CleanUpPlaceSceneNode::providedPorts() {
    return {};
}

// 该节点向服务端发送任务目标时不需要向黑板中读取任何数据
bool CleanUpPlaceSceneNode::prepareGoal(Action::Goal& goal) {
    (void)goal;
    return true;
}

// 拿到服务端返回的数据后仅需要检查任务是否成功
BT::NodeStatus CleanUpPlaceSceneNode::onResult(const WrappedResult& wrapped) {
    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}
