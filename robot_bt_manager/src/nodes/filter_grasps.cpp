#include "robot_bt_manager/nodes/filter_grasps.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会从黑板中读入 GraspNet 所有输出的评估结果,以及机械臂当前的关节角度值,以及物体的中心位置
// 该节点会向黑板中写入过滤后抓取位姿和分数,以及任务的错误码
BT::PortsList FilterGraspsNode::providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("grasp_poses"),
        BT::InputPort<std::vector<float>>("scores"),
        BT::InputPort<std::vector<float>>("widths"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose"),
        BT::InputPort<std::vector<double>>("joint_values"),

        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("filter_poses"),
        BT::OutputPort<std::vector<double>>("filter_scores"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取 GraspNet 所有输出的评估结果,以及机械臂当前的关节角度值,以及物体的中心位置
bool FilterGraspsNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中依次读入数据
    auto grasp_poses = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("grasp_poses");
    auto scores = getInput<std::vector<float>>("scores");
    auto widths = getInput<std::vector<float>>("widths");
    auto object_pose = getInput<geometry_msgs::msg::PoseStamped>("object_pose");
    auto joint_values = getInput<std::vector<double>>("joint_values");

    // 读取不到数据报错
    if (!grasp_poses || !scores || !widths || !object_pose || !joint_values) return false;

    // 读取到则准备发给服务端的数据
    goal.candidates = *grasp_poses;
    goal.scores = *scores;
    goal.widths = *widths;
    goal.object_pose = *object_pose;
    goal.joint_values = *joint_values;
    
    return true;
}

// 该节点需要向黑板写入过滤后抓取位姿和分数,以及任务的错误码
BT::NodeStatus FilterGraspsNode::onResult(const WrappedResult& wrapped) {
    // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入过滤后抓取位姿和分数
    setOutput("filter_poses", wrapped.result->filter_poses);
    setOutput("filter_scores", wrapped.result->filter_scores);
    return BT::NodeStatus::SUCCESS;
}