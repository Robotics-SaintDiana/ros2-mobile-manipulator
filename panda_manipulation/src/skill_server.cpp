#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "panda_manipulation/manipulator_executor.hpp"
#include "panda_manipulation/perception_executor.hpp"
#include "panda_manipulation/grasp_feasibility_filter.hpp"

#include "panda_manipulation/skills/get_object_center_server.hpp"
#include "panda_manipulation/skills/prepare_pick_scene_server.hpp"
#include "panda_manipulation/skills/cleanup_pick_scene_server.hpp"
#include "panda_manipulation/skills/get_fusion_cloud_server.hpp"
#include "panda_manipulation/skills/get_joint_values_server.hpp"
#include "panda_manipulation/skills/filter_grasps_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("panda_skills_server");

    // 所有的子技能共享同一份执行器
    auto manipulator_executor = std::make_shared<ManipulatorExecutor>(node);
    auto perception_executor = std::make_shared<PerceptionExecutor>(node);
    auto grasp_feasibility_filter = std::make_shared<GraspFeasibilityFilter>(node);
    auto task_executor = std::make_shared<TaskExecutor>(node, manipulator_executor, perception_executor);

    GetObjectCenterServer get_object_center(node, perception_executor);
    PreparePickSceneServer prepare_pick_scene(node, manipulator_executor);
    CleanUpPickSceneServer cleanup_pick_scene(node, manipulator_executor);
    GetFusionCloudServer get_fusion_cloud(node, task_executor);
    GetJointValuesServer get_joint_value(node, manipulator_executor);
    FilterGraspsServer filter_grasps(node, grasp_feasibility_filter);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}