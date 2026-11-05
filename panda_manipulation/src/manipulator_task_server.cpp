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
#include "panda_manipulation/grasp_feasibility_filter.hpp"

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
            feasibility_filter_ = std::make_shared<GraspFeasibilityFilter>(this->shared_from_this());
            graspnet_client_ = rclcpp_action::create_client<InferGrasps>(this, "/graspnet/infer_grasps");
        }    

    private:
        rclcpp_action::Server<ManipulateObject>::SharedPtr action_server_;
        rclcpp_action::Client<InferGrasps>::SharedPtr graspnet_client_;
        std::shared_ptr<ManipulatorExecutor> manipulator_executor_;
        std::shared_ptr<PerceptionExecutor> perception_executor_;
        std::shared_ptr<GraspFeasibilityFilter> feasibility_filter_;

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

            // 机械臂运动子任务与 GraspNet 感知子任务的结果
            auto manipulate_result = std::make_shared<ManipulateObject::Result>();
            auto grasp_result = std::make_shared<InferGrasps::Result>();
            
            // 情况一：需要执行 Pick 指令
            if (task_goal->action_type == "pick") {
                // 1. 控制机械臂获取物体的点云数据
                sensor_msgs::msg::PointCloud2 filiter_cloud;
                if (!getFusionResult(filiter_cloud)) return;

                // 2. 将获取到的最新点云数据传给 GraspNet 进行推理，获取相应的抓取位姿
                if (!getGraspnetResult(filiter_cloud, grasp_result)) return;

                // 3. 将 GraspNet 输出的推理结果进行运动学可行性过滤
                const auto current_joints = manipulator_executor_->getCurrentJointValues();
                std::vector<double> scores_double(grasp_result->scores.begin(), grasp_result->scores.end());
                const auto filterResult = feasibility_filter_->graspPoseFilter(
                    grasp_result->grasp_poses, scores_double, current_joints);
                if (filterResult.empty()) {
                    RCLCPP_ERROR(this->get_logger(),"[PICK] no feasible grasp");
                    return;
                }

                // 4. 按得分从高到低尝试执行，最多尝试前 3 个
                const size_t max_try = std::min(filterResult.size(), size_t{3});
                bool is_execute_success = false;

                for (size_t i = 0; i < max_try; ++i) {
                    if (manipulator_executor_->pickExecutor(filterResult[i].pose)) {
                        is_execute_success = true;
                        break;          
                    }
                }

                if (!is_execute_success) {
                    RCLCPP_ERROR(this->get_logger(), "[PICK] Execute Failed!");
                    return;
                }
            } 
            
            // 情况二：需要执行 Place 指令
            if (task_goal->action_type == "place") {
                geometry_msgs::msg::PoseStamped place_pose;
                if (!perception_executor_->getPlatePosition("green", place_pose.pose)) return;

                place_pose.header.frame_id = "panda_link0";
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

            // 2. 将放置物体的桌面加入 Moveit 的规划场景，防止机械臂在运动时与桌面发生碰撞
            if (!manipulator_executor_->prepareCollisionScene(
                    object_pose.pose.position.x, object_pose.pose.position.y)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to apply table collision object!");
                return false;
            }

            // 3. 根据顶部相机获取的位置计算三次拍摄中，机械臂应该到达的位置
            // 3.1 定义机械臂到达位置数组，默认数组大小为 3
            std::vector<geometry_msgs::msg::PoseStamped> arm_poses;
            arm_poses.resize(3);

            // 3.2 获取手部相机光学坐标系与 panda_hand 的 tf 变换关系
            tf2::Transform camera_in_hand;
            if (!manipulator_executor_->getTransform("panda_hand", "hand_camera_depth_optical_frame", camera_in_hand)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get hand camera extrinsics!");
                return false;
            }

            // 3.3 获取机械臂到达位置
            getObservationPoses(arm_poses, object_pose, camera_in_hand);

            // 4. 控制机械臂运动至指定位置，每运动到一个地方就获取一次点云数据
            std::vector<sensor_msgs::msg::PointCloud2> object_clouds;
            for (int i = 0; i < 3; i++) {  
                // 先控制机械臂运动到指定的位置，因为运动是阻塞式的，只有全部完成了才会返回 
                // 从第 2 个位置去第 3 个位置时，先回到中间过渡点，防止运动规划失败或者碰到物体
                if (i == 2) {
                    if (!manipulator_executor_->armMoveExecutor(arm_poses[0])) {
                        return false;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                }

                // 运动到当前目标位置
                if (!manipulator_executor_->armMoveExecutor(arm_poses[i])) return false;

                // 稍微延时一下，等待仿真稳定且获取到了最新的点云数据
                rclcpp::sleep_for(std::chrono::milliseconds(500));

                // 获取点云数据
                sensor_msgs::msg::PointCloud2 hand_camera_cloud;
                if (!perception_executor_->getHandCameraLatestProcessedCloud(hand_camera_cloud)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to capture point cloud at pose %d", i);
                    return false;
                }

                // 拍摄完成后让机械臂回到 Home 点，方便后续的抓取操作
                if (!manipulator_executor_->moveHome()) return false;
                
                object_clouds.push_back(hand_camera_cloud);
            }

            // 5. 将点云数据进行融合
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
        void getObservationPoses(std::vector<geometry_msgs::msg::PoseStamped>& arm_poses, const geometry_msgs::msg::PoseStamped& object_pose, const tf2::Transform& camera_in_hand) {
             const tf2::Vector3 object(object_pose.pose.position.x,
                                  object_pose.pose.position.y,
                                  object_pose.pose.position.z);

            const double tilt = 45.0 * M_PI / 180.0;      // 侧视角倾角
            const double cam_distance = 0.35;             // 相机到物体的距离

            // 相机外参常量
            const tf2::Vector3    p_cam_in_hand = camera_in_hand.getOrigin();
            const tf2::Matrix3x3  R_cam_in_hand = camera_in_hand.getBasis();
            // "手原点相对相机原点"的向量，表达在相机坐标系里
            const tf2::Vector3    hand_in_cam = R_cam_in_hand.transpose() * p_cam_in_hand;
            // panda_link8 <- panda_hand 的固定旋转
            const tf2::Matrix3x3  R_link8_in_hand(
                tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -2.35619449));

            std::vector<tf2::Vector3> view_dirs;
            view_dirs.push_back(tf2::Vector3(0.0, 0.0, 1.0));
            view_dirs.push_back(tf2::Vector3(0.0,  std::sin(tilt), std::cos(tilt)));
            view_dirs.push_back(tf2::Vector3(0.0, -std::sin(tilt), std::cos(tilt)));

            for (std::size_t i = 0; i < arm_poses.size() && i < view_dirs.size(); ++i) {
                tf2::Vector3 u = view_dirs[i];
                u.normalize();                       // 物体 -> 相机的单位方向

                // 1) 理想相机姿态：光轴 z = -u，"画面上方"尽量对齐世界的 +z
                tf2::Vector3 z_c(-u.x(), -u.y(), -u.z());
                tf2::Vector3 x_c = tf2::Vector3(0.0, 0.0, 1.0).cross(z_c);
                if (x_c.length() < 1e-3) x_c = tf2::Vector3(1.0, 0.0, 0.0);
                x_c.normalize();
                tf2::Vector3 y_c = z_c.cross(x_c);
                const tf2::Matrix3x3 R_cam(
                    x_c.x(), y_c.x(), z_c.x(),
                    x_c.y(), y_c.y(), z_c.y(),
                    x_c.z(), y_c.z(), z_c.z());

                // 2) 理想相机位置 + 反推手原点
                const tf2::Vector3 cam_pos(
                    object.x() + u.x() * cam_distance,
                    object.y() + u.y() * cam_distance,
                    object.z() + u.z() * cam_distance);
                const tf2::Vector3 off = R_cam * hand_in_cam;
                const tf2::Vector3 hand_pos(cam_pos.x() - off.x(),
                                            cam_pos.y() - off.y(),
                                            cam_pos.z() - off.z());

                // 3) 手的姿态 -> link8 的姿态
                const tf2::Matrix3x3 R_hand = R_cam * R_cam_in_hand.transpose();
                const tf2::Matrix3x3 R_link8 = R_hand * R_link8_in_hand;

                tf2::Quaternion q;
                R_link8.getRotation(q);

                arm_poses[i].pose.position.x = hand_pos.x();
                arm_poses[i].pose.position.y = hand_pos.y();
                arm_poses[i].pose.position.z = hand_pos.z();
                arm_poses[i].pose.orientation = tf2::toMsg(q);
                arm_poses[i].header.frame_id = "panda_link0";
                arm_poses[i].header.stamp = object_pose.header.stamp;
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