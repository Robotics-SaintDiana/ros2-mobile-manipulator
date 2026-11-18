#include "robot_bt_manager/nodes/prepare_place_scene.hpp"

// 该节点会从黑板中读取托盘的中心位置
BT::PortsList PreparePlaceSceneNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取托盘的中心位置坐标
bool PreparePlaceSceneNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中读取托盘的中心位置坐标
    auto plate_pose = getInput<geometry_msgs::msg::PoseStamped>("place_pose");

    // 读取不到数据报错，读取到则准备发给服务端的数据
    if (!plate_pose) return false;

    goal.plate_pose = *plate_pose;
    return true;
}

// 拿到服务端返回的数据后仅需要检查任务是否成功
BT::NodeStatus PreparePlaceSceneNode::onResult(const WrappedResult& wrapped) {
    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}