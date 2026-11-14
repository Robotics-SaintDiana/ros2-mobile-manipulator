#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "panda_manipulation/manipulator_executor.hpp"
#include "panda_manipulation/perception_executor.hpp"

#include "panda_manipulation/skills/move_home_server.hpp"
#include "panda_manipulation/skills/get_object_center.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("panda_skills_server");

    // 两个技能共享同一份执行器
    auto manipulator = std::make_shared<ManipulatorExecutor>(node);
    auto perception  = std::make_shared<PerceptionExecutor>(node);

    MoveHomeServer move_home(node, manipulator);
    GetObjectCenterServer get_center(node, perception);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}