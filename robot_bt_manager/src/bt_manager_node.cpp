#include <memory>
#include <exception>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_manipulation_msgs/msg/error_code.hpp>

#include "robot_manipulation_msgs/action/execute_bt_task.hpp"

#include "robot_bt_manager/nodes/navigate_to_pose.hpp"
#include "robot_bt_manager/nodes/get_object_center.hpp"
#include "robot_bt_manager/nodes/prepare_pick_scene.hpp"
#include "robot_bt_manager/nodes/cleanup_pick_scene.hpp"
#include "robot_bt_manager/nodes/get_fusion_cloud.hpp"
#include "robot_bt_manager/nodes/infer_grasps.hpp"
#include "robot_bt_manager/nodes/get_joint_values.hpp"
#include "robot_bt_manager/nodes/filter_grasps.hpp"
#include "robot_bt_manager/nodes/execute_grasp.hpp"
#include "robot_bt_manager/nodes/get_plate_position.hpp"
#include "robot_bt_manager/nodes/prepare_place_scene.hpp"
#include "robot_bt_manager/nodes/cleanup_place_scene.hpp"
#include "robot_bt_manager/nodes/execute_place.hpp"

class ExecuteBTTaskServer : public rclcpp::Node
{
    public:
        using ExecuteBTTask = robot_manipulation_msgs::action::ExecuteBTTask;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBTTask>;
        using ErrorCode = robot_manipulation_msgs::msg::ErrorCode;

        explicit ExecuteBTTaskServer() : Node("bt_manager_node") {
            // 注册 BT 节点
            registerBTNodes();

            // 获取 BT 树 XML 文件地址
            const auto share = ament_index_cpp::get_package_share_directory("robot_bt_manager");
            tree_path_ = share + "/behavior_trees/pick_and_place.xml";
            
            using namespace std::placeholders;
            action_server_ = rclcpp_action::create_server<ExecuteBTTask>(
                this,
                "/manipulator/execute_bt_task",
                std::bind(&ExecuteBTTaskServer::handle_goal, this, _1, _2),
                std::bind(&ExecuteBTTaskServer::handle_cancel, this, _1),
                std::bind(&ExecuteBTTaskServer::handle_accepted, this, _1)
            );
        }

    private:
        rclcpp_action::Server<ExecuteBTTask>::SharedPtr action_server_;
        BT::BehaviorTreeFactory factory_;
        std::string tree_path_;
        
        void registerBTNodes() {
            // 注册 Nav2 的 BT 节点
            factory_.registerBuilder<NavigateToPoseNode>("NavigateToPose",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<NavigateToPoseNode>(name, config, this->shared_from_this());
            });

            // 注册 Pick 流程中的主要 BT 节点
            factory_.registerBuilder<GetObjectCenterNode>("GetObjectCenter",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<GetObjectCenterNode>(name, config, this->shared_from_this());
            });

             factory_.registerBuilder<GetFusionCloudNode>("GetFusionCloud",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<GetFusionCloudNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<InferGraspsNode>("InferGrasps",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<InferGraspsNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<GetJointValuesNode>("GetJointValues",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<GetJointValuesNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<FilterGraspsNode>("FilterGrasps",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<FilterGraspsNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<ExecuteGraspNode>("ExecuteGrasp",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<ExecuteGraspNode>(name, config, this->shared_from_this());
            });

            // 2. 注册 Place 流程中的主要 BT 节点
            factory_.registerBuilder<GetPlatePositionNode>("GetPlatePosition",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<GetPlatePositionNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<ExecutePlaceNode>("ExecutePlace",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<ExecutePlaceNode>(name, config, this->shared_from_this());
            });

            // 3, 其他剩余的辅助 BT 节点
            factory_.registerBuilder<PreparePickSceneNode>("PreparePickScene",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<PreparePickSceneNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<CleanUpPickSceneNode>("CleanUpPickScene",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<CleanUpPickSceneNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<PreparePlaceSceneNode>("PreparePlaceScene",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<PreparePlaceSceneNode>(name, config, this->shared_from_this());
            });

            factory_.registerBuilder<CleanUpPlaceSceneNode>("CleanUpPlaceScene",
                [this](const std::string& name, const BT::NodeConfiguration& config) {
                    return std::make_unique<CleanUpPlaceSceneNode>(name, config, this->shared_from_this());
            });
        }

        // 目标接收回调
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteBTTask::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // 取消请求回调
        rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 目标被接受后的回调
        void handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
            // 另外开一个线程执，防止堵塞主线程
            std::thread([this, goal_handle]() { 
                execute(goal_handle); 
            }).detach();
        }

        void execute(std::shared_ptr<GoalHandle> goal_handle) {
             // 构造服务器返回的结果和反馈(依据自定义的 action 消息)
            auto result = std::make_shared<ExecuteBTTask::Result>();
            auto feedback = std::make_shared<ExecuteBTTask::Feedback>();

            // 任务处于规划阶段，反馈给客户端
            feedback->stage = "executing";
            goal_handle->publish_feedback(feedback);

            // 获取 task_manager 解析的 nav2 目标位置，以及相应的托盘颜色
            const auto goal = goal_handle->get_goal();
            auto pick_position = goal->pick_position;
            auto place_position = goal->place_position;
            auto plate_color = goal->plate_color;

            // 创建 BT 
            std::unique_ptr<BT::Tree> tree_;
            try {
                tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_path_));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load behavior tree [%s]: %s", tree_path_.c_str(), e.what());
                return;
            }

            // 将上述信息写入黑板，供后续的 BT 使用
            auto bt_blackboard = tree_->rootBlackboard();
            bt_blackboard->set("pick_position", pick_position);
            bt_blackboard->set("place_position", place_position);
            bt_blackboard->set("plate_color", plate_color);

            // 开始循环 tick BT(暂时不考虑 cancel 的情况，后续补充)
            rclcpp::Rate rate(10.0);
            BT::NodeStatus status = BT::NodeStatus::RUNNING;

            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
                status = tree_->tickRoot();
                rate.sleep();
            }

            // 返回结果
             if (status == BT::NodeStatus::SUCCESS) {
                result->success = true;
                result->error_code = ErrorCode::SUCCESS;    
                result->message = "BT task succeeded";
                goal_handle->succeed(result);
            } else if (status == BT::NodeStatus::FAILURE) {
                result->success = false;
                result->error_code = ErrorCode::SKILL_FAILED;
                result->message = "BT task failed";
                goal_handle->abort(result);
            } else {
                result->success = false;
                result->error_code = ErrorCode::CANCEL;
                result->message = "BT task interrupted";
                goal_handle->abort(result);
            }
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExecuteBTTaskServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}