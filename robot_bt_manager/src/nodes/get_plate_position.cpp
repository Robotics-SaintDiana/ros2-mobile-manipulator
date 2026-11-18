#include "robot_bt_manager/nodes/get_plate_position.hpp"
#include <robot_manipulation_msgs/msg/error_code.hpp>

using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

// 该节点会向黑板中写入指定颜色托盘的位姿数据及任务的错误码
BT::PortsList GetPlatePositionNode::providedPorts() {
    return {
        BT::InputPort<std::string>("plate_color"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("place_pose"),
        BT::OutputPort<int>("error_code"),
    };
}

// 该节点向服务端发送任务目标时需要从黑板中读取需要抓取的托盘的颜色
bool GetPlatePositionNode::prepareGoal(Action::Goal& goal) {
    // 从黑板中读取需要抓取的托盘的颜色
    auto plate_color = getInput<std::string>("plate_color");

    // 读取不到数据报错，读取到则准备发给服务端的数据
    if (!plate_color) return false;

    goal.plate_color = *plate_color;
    return true;
}

// 拿到服务端返回的数据后，该节点需要向黑板写入需要抓取托盘的中心位置以及错误码
BT::NodeStatus GetPlatePositionNode::onResult(const WrappedResult& wrapped) {
    // 向黑板中写入错误码
    setOutput("error_code", wrapped.result->error_code);

    // 检查服务端中进行的任务是否成功
    if (!wrapped.result->success) return BT::NodeStatus::FAILURE;

    // 成功后向黑板中写入物体的中心位置数据
    setOutput("place_pose", wrapped.result->place_pose);
    return BT::NodeStatus::SUCCESS;
}