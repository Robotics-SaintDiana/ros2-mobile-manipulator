#include <memory>
#include <exception>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "robot_bt_manager/nodes/navigate_to_pose.hpp"
#include "robot_bt_manager/nodes/get_object_center.hpp"
#include "robot_bt_manager/nodes/prepare_pick_scene.hpp"
#include "robot_bt_manager/nodes/cleanup_pick_scene.hpp"
#include "robot_bt_manager/nodes/get_fusion_cloud.hpp"
#include "robot_bt_manager/nodes/infer_grasps.hpp"
#include "robot_bt_manager/nodes/get_joint_values.hpp"
#include "robot_bt_manager/nodes/filter_grasps.hpp"

int main(int argc, char** argv)
{
    // 创建 ROS2 节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_manager_node");

    // 注册 BT 节点
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<NavigateToPoseNode>("NavigateToPose",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavigateToPoseNode>(name, config, node);
    });

    factory.registerBuilder<GetObjectCenterNode>("GetObjectCenter",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetObjectCenterNode>(name, config, node);
    });

    factory.registerBuilder<PreparePickSceneNode>("PreparePickScene",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<PreparePickSceneNode>(name, config, node);
    });

    factory.registerBuilder<CleanUpPickSceneNode>("CleanUpPickScene",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CleanUpPickSceneNode>(name, config, node);
    });

    factory.registerBuilder<GetFusionCloudNode>("GetFusionCloud",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetFusionCloudNode>(name, config, node);
    });

    factory.registerBuilder<InferGraspsNode>("InferGrasps",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<InferGraspsNode>(name, config, node);
    });

    factory.registerBuilder<GetJointValuesNode>("GetJointValues",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetJointValuesNode>(name, config, node);
    });

    factory.registerBuilder<FilterGraspsNode>("FilterGrasps",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<FilterGraspsNode>(name, config, node);
    });

    const auto share = ament_index_cpp::get_package_share_directory("robot_bt_manager");
    const auto tree_path = share + "/behavior_trees/pick_and_place.xml";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(tree_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree [%s]: %s", tree_path.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::Rate rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Behavior tree finished: SUCCESS");
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(node->get_logger(), "Behavior tree finished: FAILURE");
    } else {
        RCLCPP_WARN(node->get_logger(), "Behavior tree stopped: %d",
                    static_cast<int>(status));
    }

    rclcpp::shutdown();

    return 0;
}