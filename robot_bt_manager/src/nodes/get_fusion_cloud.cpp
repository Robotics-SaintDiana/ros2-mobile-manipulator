#include "robot_bt_manager/nodes/get_fusion_cloud.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会从黑板中读入物体的中心位置坐标
// 该节点会向黑板中写入融合后的点云数据及任务的错误码
BT::PortsList GetFusionCloudNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose"),
        BT::OutputPort<sensor_msgs::msg::PointCloud2>("fusion_cloud"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取物体的中心位置坐标
bool GetFusionCloudNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中读取物体的中心位置坐标
    auto object_pose = getInput<geometry_msgs::msg::PoseStamped>("object_pose");

    // 读取不到数据报错，读取到则准备发给服务端的数据
    if (!object_pose) return false;

    goal.object_pose = *object_pose;
    return true;
}

// 该节点需要向黑板写入错误码，以便 BT 根据错误码进行重试等功能
BT::NodeStatus GetFusionCloudNode::onResult(const WrappedResult& wrapped) {
    // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入融合后的点云数据
    setOutput("fusion_cloud", wrapped.result->fusion_cloud);
    return BT::NodeStatus::SUCCESS;
}