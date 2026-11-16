#include "robot_bt_manager/nodes/infer_grasps.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会从黑板中读入融合后的点云数据
// 该节点会向黑板中写入推理后获得的抓取位姿及相应的得分和夹爪宽度，以及错误码
BT::PortsList InferGraspsNode::providedPorts() {
    return {
        BT::InputPort<sensor_msgs::msg::PointCloud2>("fusion_cloud"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("grasp_poses"),
        BT::OutputPort<std::vector<float>>("scores"),
        BT::OutputPort<std::vector<float>>("widths"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取融合后的点云数据
bool InferGraspsNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中读取融合后的点云数据
    auto fusion_cloud = getInput<sensor_msgs::msg::PointCloud2>("fusion_cloud");

    // 读取不到数据报错，读取到则准备发给服务端的数据
    if (!fusion_cloud) return false;

    goal.cloud = *fusion_cloud;
    return true;
}

// 该节点需要向黑板写入推理后获得的抓取位姿及相应的得分和夹爪宽度，以及错误码
BT::NodeStatus InferGraspsNode::onResult(const WrappedResult& wrapped) {
    // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入推理后获得的抓取位姿及相应的得分和夹爪宽度
    setOutput("grasp_poses", wrapped.result->grasp_poses);
    setOutput("scores", wrapped.result->scores);
    setOutput("widths", wrapped.result->widths);
      
    return BT::NodeStatus::SUCCESS;
}