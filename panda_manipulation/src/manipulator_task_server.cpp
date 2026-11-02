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
                // 1. 控制机械臂获取物体的点云数据
                sensor_msgs::msg::PointCloud2 filiter_cloud;
                if (!getFusionResult(filiter_cloud)) return;

                // 2. 将获取到的最新点云数据传给 graspnet 进行推理，获取相应的抓取位姿
                if (!getGraspnetResult(filiter_cloud, grasp_result)) return;

                geometry_msgs::msg::PoseStamped pick_pose;
                pick_pose.pose.position = grasp_result->grasp_poses[0].pose.position;
                pick_pose.pose.orientation = grasp_result->grasp_poses[0].pose.orientation;
                pick_pose.header.frame_id = "hand_camera_depth_optical_frame";
                pick_pose.header.stamp = this->now();
                // 3. 测试：先将分数最高的抓取位姿交由机械臂执行(执行前其实应该检验一下数据是否正确，这部分可以放在机械臂中检验)
                if (!manipulator_executor_->pickExecutor(pick_pose)) return;
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

        // -------------------------------------
        // ------------ 辅助函数部分 -------------
        // -------------------------------------
        // 控制机械臂在三个不同的位置获取三份点云数据，并将这些点云数据进行融合
        bool getFusionResult(sensor_msgs::msg::PointCloud2& filiter_cloud) {
            // 1. 利用顶部相机获取目标物体的大致位置
            geometry_msgs::msg::PoseStamped object_pose;
            if (!perception_executor_->getObjectCenterPosition(object_pose)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to calculate object's center position!");
                return false;
            }

            // 2. 根据顶部相机获取的位置计算三次拍摄中，机械臂应该到达的位置
            std::vector<geometry_msgs::msg::PoseStamped> arm_poses;
            arm_poses.resize(3);
            getObservationPoses(arm_poses, object_pose);

            // 3. 控制机械臂运动至指定位置，每运动到一个地方就获取一次点云数据
            std::vector<sensor_msgs::msg::PointCloud2> object_clouds;
            for (int i = 0; i < 3; i++) {  
                // 先控制机械臂运动到指定的位置，因为运动是阻塞式的，只有全部完成了才会返回 
                if (!manipulator_executor_->armMoveExecutor(arm_poses[i])) return false;

                // 稍微延时一下，等待仿真稳定且获取到了最新的点云数据
                rclcpp::sleep_for(std::chrono::seconds(2));

                // 获取点云数据
                sensor_msgs::msg::PointCloud2 hand_camera_cloud;
                if (!perception_executor_->getHandCameraLatestProcessedCloud(hand_camera_cloud)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to capture point cloud at pose %d", i);
                    return false;
                }

                object_clouds.push_back(hand_camera_cloud);
            }

            // 4. 将点云数据进行融合
            filiter_cloud = perception_executor_->getHandCameraFusionCloud(object_clouds);
            if (filiter_cloud.data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Hand camera fusion cloud is empty!");
                return false;
            }

            return true;
        }


        // 向 GraspNet 中传入点云数据，获取 GraspNet 返回的推理结果
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

        // 根据相机获取到的物体中心位置，生成相机拍摄的三个位姿
        void getObservationPoses(std::vector<geometry_msgs::msg::PoseStamped>& arm_poses, const geometry_msgs::msg::PoseStamped& object_pose) {
            // 提取中心坐标
            double cx = object_pose.pose.position.x;
            double cy = object_pose.pose.position.y;
            double cz = object_pose.pose.position.z;

            // 定义拍摄时的安全高度偏移
            double standoff_z = 0.35;                    // 在 Z 轴上，相机距离物体的安全高度
            double x_offset = 0.2;                      // 左右两个视角相对于正上方俯视视角，在左右上偏置 15cm
            double y_offset = 0.05;
            double tilt_angle = 45.0 * M_PI / 180.0;     // 左右两个视角相对于正上方俯视视角，在 Z 轴上倾斜 30 度向内看

            // -----------------------------------------------------------------
            // 1. 中间视角（Center）：正上方俯视
            // -----------------------------------------------------------------
            // 仅在竖直方向上向上抬升，仍然保持向下的俯视姿态
            arm_poses[0].pose.position.x = cx;
            arm_poses[0].pose.position.y = cy;
            arm_poses[0].pose.position.z = cz - standoff_z;

            tf2::Quaternion q_center;
            q_center.setRPY(0.0, 0.0, M_PI / 2);
            arm_poses[0].pose.orientation = tf2::toMsg(q_center);

            // -----------------------------------------------------------------
            // 2. 左侧视角（Left）：位于左侧，向右倾斜 30 度
            // -----------------------------------------------------------------
            arm_poses[1].pose.position.x = cx + x_offset;
            arm_poses[1].pose.position.y = cy - y_offset;
            arm_poses[1].pose.position.z = cz - standoff_z;

            tf2::Quaternion q_left;
            q_left.setRPY(0.0, -tilt_angle + M_PI / 2, -M_PI);
            arm_poses[1].pose.orientation = tf2::toMsg(q_left);

            // -----------------------------------------------------------------
            // 3. 右侧视角（Right）：位于右侧，向左倾斜 30 度
            // -----------------------------------------------------------------
            arm_poses[2].pose.position.x = cx - x_offset;
            arm_poses[2].pose.position.y = cy - y_offset;
            arm_poses[2].pose.position.z = cz - standoff_z;

            tf2::Quaternion q_right;
            q_right.setRPY(0.0, tilt_angle, 0.0);
            arm_poses[2].pose.orientation = tf2::toMsg(q_right);

            for (auto& arm_pose : arm_poses) {
                arm_pose.header.frame_id = object_pose.header.frame_id;
            }
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