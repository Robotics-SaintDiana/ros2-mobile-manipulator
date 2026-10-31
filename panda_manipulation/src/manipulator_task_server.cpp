#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"

#include "robot_manipulation_msgs/action/manipulate_object.hpp"
#include "robot_manipulation_msgs/action/infer_grasps.hpp"
#include "panda_manipulation/manipulator_executor.hpp"
#include "panda_manipulation/perception_executor.hpp"

class ManipulatorTaskServer : public rclcpp::Node
{
    public:
        using ManipulateObject = robot_manipulation_msgs::action::ManipulateObject;
        using InferGrasps = robot_manipulation_msgs::action::InferGrasps;
        using GoalHandleManipulateObject = rclcpp_action::ServerGoalHandle<ManipulateObject>;
        using GoalHandleInferGrasps = rclcpp_action::ServerGoalHandle<InferGrasps>;

        // 初始化函数 --- 构造 Manipulator Action Server
        explicit ManipulatorTaskServer() : Node("manipulator_task_server") {
            using namespace std::placeholders;

            // handle_goal     ： 决定是否接收客户端任务
            // handle_cancel   ： 客户端取消任务时调用
            // handle_accepted ： 接受后客户端的执行回调  
            action_server_ = rclcpp_action::create_server<ManipulateObject>(
                this,
                "/manipulator/execute_task",
                std::bind(&ManipulatorTaskServer::handle_goal, this, _1, _2),
                std::bind(&ManipulatorTaskServer::handle_cancel, this, _1),
                std::bind(&ManipulatorTaskServer::handle_accepted, this, _1)
            );
        }

        // 初始化函数 --- 初始化子任务执行器，以及 graspnet 的客户端
        void initialize() {
            manipulator_executor_ = std::make_shared<ManipulatorExecutor>(this->shared_from_this());
            perception_executor_ = std::make_shared<PerceptionExecutor>(this->shared_from_this());
            graspnet_client_ = rclcpp_action::create_client<InferGrasps>(this, "/graspnet/infer_grasps");
        }    

    private:
        rclcpp_action::Server<ManipulateObject>::SharedPtr action_server_;
        rclcpp_action::Client<InferGrasps>::SharedPtr graspnet_client_;
        std::shared_ptr<ManipulatorExecutor> manipulator_executor_;
        std::shared_ptr<PerceptionExecutor> perception_executor_;

        // 1. 决定是否接收来自客户端的任务
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const ManipulateObject::Goal> goal)
        {
            (void)uuid;

            RCLCPP_INFO(
                this->get_logger(),
                "Received goal: action_type='%s', object_id='%s', color='%s'",
                goal->action_type.c_str(),
                goal->object_id.c_str(),
                goal->color.c_str()
            );
            
            // 只执行 Pick 和 Place 两种类型的任务
            if (goal->action_type != "pick" && goal->action_type != "place") {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Rejecting unsupported action_type: '%s'",
                    goal->action_type.c_str()
                );
                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // 2. Handle a cancellation request.
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleManipulateObject> goal_handle)
        {
            (void)goal_handle;

            RCLCPP_INFO(this->get_logger(), "Received request to cancel the manipulation task.");

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 3. Start executing an accepted goal.
        void handle_accepted(const std::shared_ptr<GoalHandleManipulateObject> goal_handle) {
            // Execute the long-running task outside the executor callback.
            std::thread(
                std::bind(&ManipulatorTaskServer::execute, this, goal_handle)
            ).detach();
        }

        // Publish the current execution stage.
        void publish_stage(const std::shared_ptr<GoalHandleManipulateObject> goal_handle, const std::string & stage) {
            auto feedback = std::make_shared<ManipulateObject::Feedback>();

            feedback->stage = stage;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Task stage: %s", stage.c_str());
        }

        // Check whether the client has requested cancellation.
        bool handle_if_canceling(
            const std::shared_ptr<GoalHandleManipulateObject> goal_handle)
        {
            if (!goal_handle->is_canceling()) {
                return false;
            }

            auto result = std::make_shared<ManipulateObject::Result>();
            result->success = false;
            result->message = "Task canceled by client.";

            goal_handle->canceled(result);

            RCLCPP_INFO(this->get_logger(), "Manipulation task canceled.");

            return true;
        }

        // 4. 执行任务管理器发送的 Pick 和 Place 指令，并向上返回相应的结果
        void execute(const std::shared_ptr<GoalHandleManipulateObject> goal_handle) {
            // 获取任务管理器发送的指令目标
            const auto task_goal = goal_handle->get_goal();

            // 机械臂运动子任务与 Graspnet 感知子任务的结果
            auto manipulate_result = std::make_shared<ManipulateObject::Result>();
            auto grasp_result = std::make_shared<InferGrasps::Result>();
            
            // 情况一：需要执行 Pick 指令
            if (task_goal->action_type == "pick") {
                // // 1. 先获取手部相机的点云数据
                // sensor_msgs::msg::PointCloud2 hand_camera_lasted_cloud;
                // if (!perception_executor_->getHandCameraLatestProcessedCloud(hand_camera_lasted_cloud)) {
                //     return;
                // }

                // // 2. 将获取到的最新点云数据传给 graspnet 进行推理，获取相应的抓取位姿
                // if (!getGraspnetResult(hand_camera_lasted_cloud, grasp_result)) return;

                // geometry_msgs::msg::PoseStamped pick_pose;
                // pick_pose.pose.position = grasp_result->grasp_poses[0].pose.position;
                // pick_pose.pose.orientation = grasp_result->grasp_poses[0].pose.orientation;
                // pick_pose.header.frame_id = "hand_camera_depth_optical_frame";
                // pick_pose.header.stamp = this->now();
                // // 3. 测试：先将分数最高的抓取位姿交由机械臂执行(执行前其实应该检验一下数据是否正确，这部分可以放在机械臂中检验)
                // if (!manipulator_executor_->pickExecutor(pick_pose)) return;
            } 
            
            // 情况二：需要执行 Place 指令
            if (task_goal->action_type == "place") {
                geometry_msgs::msg::PoseStamped place_pose;
                if (!perception_executor_->getPlatePosition("green", place_pose.pose)) return;

                place_pose.header.frame_id = "hand_camera_depth_optical_frame";
                place_pose.header.stamp = this->now();
                if (!manipulator_executor_->placeExecutor(place_pose)) return;
            }
            
            manipulate_result->success = true;
            manipulate_result->message = "Mock " + task_goal->action_type + " task completed successfully.";

            goal_handle->succeed(manipulate_result);
        }

        bool getGraspnetResult(const sensor_msgs::msg::PointCloud2& cloud, std::shared_ptr<InferGrasps::Result>& grasp_result) {
            // 1. 检查 Graspnet 服务器是否准备完毕
            if (!graspnet_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(get_logger(), "GraspNet Action Server is not available.");
                return false;
            }

            // 2. 向 Graspnet 服务器进行发送的点云数据，并等待服务器接受
            InferGrasps::Goal grasp_goal;
            grasp_goal.cloud = cloud;

            auto send_grasp_goal_options = rclcpp_action::Client<InferGrasps>::SendGoalOptions();
            auto grasp_goal_future = graspnet_client_->async_send_goal(grasp_goal, send_grasp_goal_options);
            if (grasp_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for GraspNet goal response.");
                return false;
            }

            // 3. 获取服务器返回的句柄
            auto grasp_goal_handle = grasp_goal_future.get();
            if (!grasp_goal_handle) {
                RCLCPP_ERROR(get_logger(), "GraspNet rejected the goal.");
                return false;
            }

            // 4. 启动 graspnet 推理，等待 10s 等待返回的结果
            auto grasp_result_future = graspnet_client_->async_get_result(grasp_goal_handle);
            if (grasp_result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for GraspNet inference result.");
                return false;
            }

            // 5. 检查此次通信是否成功
            auto grasp_wrapped_result = grasp_result_future.get();
            if (grasp_wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(get_logger(), "GraspNet action did not succeed.");
                return false;
            }

            // 6. 检查 Graspnet 推理过程是否成功
            auto graspnet_inference_result = grasp_wrapped_result.result;
            if (!graspnet_inference_result || !graspnet_inference_result->success) {
                RCLCPP_ERROR(get_logger(), "GraspNet inference failed: %s", graspnet_inference_result ? graspnet_inference_result->message.c_str() : "Empty result");
                return false;
            }

            // 7. 获取 Graspnet 的推理结果
            grasp_result->grasp_poses = graspnet_inference_result->grasp_poses;
            grasp_result->scores = graspnet_inference_result->scores;
            return true;
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorTaskServer>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}