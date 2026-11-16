#include "panda_manipulation/task_executor.hpp"

// 构造函数实现
TaskExecutor::TaskExecutor(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ManipulatorExecutor> manipulator,
                           std::shared_ptr<PerceptionExecutor> perception)
    : node_(std::move(node)), manipulator_executor_(std::move(manipulator)), perception_executor_(std::move(perception)) {}

// ----- 获取目标物体的多角度融合点云 -----
TaskExecutor::GetFusionCloudStatus TaskExecutor::captureFusionCloud(geometry_msgs::msg::PoseStamped& object_pose, sensor_msgs::msg::PointCloud2& fusion_cloud) {
    // 1. 根据顶部相机获取的位置计算三次拍摄中，机械臂应该到达的位置
    // 3.1 定义机械臂到达位置数组，默认数组大小为 3
    std::vector<geometry_msgs::msg::PoseStamped> arm_poses;
    arm_poses.resize(3);

    // 1.2 获取手部相机光学坐标系与 panda_hand 的 tf 变换关系
    tf2::Transform camera_in_hand;
    if (!manipulator_executor_->getTransform("panda_hand", "hand_camera_depth_optical_frame", camera_in_hand)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get hand camera extrinsics!");
        return GetFusionCloudStatus::GET_TF_FAILED;
    }

    // 此处存在的情况为：机械臂无法到达某个指定位置；点云融合失败
    // 针对机械臂规划问题，采取跳过不执行策略，最终只要有两个位置能够到达且采集到了点云即判定成功
    // 针对点云融合失败，采用变更机械臂位置点的策略重新采集点云，最多尝试两次
    std::vector<sensor_msgs::msg::PointCloud2> object_clouds;
    for (int attempt = 0; attempt < 2; attempt++) {
        // 清除点云数据，防止重试的过程中夹杂了之前的数据
        object_clouds.clear();

        // 1.3 获取机械臂到达位置
        getObservationPoses(arm_poses, object_pose, camera_in_hand, attempt);

        // 2. 控制机械臂运动至指定位置，每运动到一个地方就获取一次点云数据
        for (int i = 0; i < 3; i++) {
            // 从第 2 个位置去第 3 个位置时，由于跨度较大，容易出现规划失败问题
            // 这里先回到中间过渡点，如果无法回归则直接尝试前往第 3 个位置
            if (i == 2) {
                if (!manipulator_executor_->armMoveExecutor(arm_poses[0])) {
                    RCLCPP_WARN(node_->get_logger(), "Transition pose failed, go directly to view 2");
                } else {
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                }
            }

            // 其他情况下直接控制机械臂运动到指定位置即可，若规划失败则再尝试一次，均失败则跳过此次运动
            const auto move_start_stamp = node_->now();
            if (!manipulator_executor_->armMoveExecutor(arm_poses[i])) {
                RCLCPP_WARN(node_->get_logger(), "View %d plan failed, retry once", i);
                if (!manipulator_executor_->armMoveExecutor(arm_poses[i])) {
                    RCLCPP_WARN(node_->get_logger(), "View %d plan failed twice, skip", i);
                    continue;
                }
            }

            // 稍微延时一下，等待仿真稳定且获取到了最新的点云数据
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            // 获取点云数据
            sensor_msgs::msg::PointCloud2 hand_camera_cloud;
            if (!perception_executor_->getHandCameraLatestProcessedCloud(hand_camera_cloud, move_start_stamp)) {
                RCLCPP_WARN(node_->get_logger(), "Failed to capture point cloud at pose %d, skip", i);
                continue;
            }

            object_clouds.push_back(hand_camera_cloud);
        }

        // 拍摄完成后让机械臂回到 Home 点，方便后续的操作
        if (!manipulator_executor_->moveHome()) {
            RCLCPP_WARN(node_->get_logger(), "Back home timeout, retry once");
            if (!manipulator_executor_->moveHome()) {
                RCLCPP_WARN(node_->get_logger(), "Back home timeout twice, skip");
            }
        }

        // 如果获取到了至少两个视角的点云，即认为任务成功可以融合
        if (object_clouds.size() >= 2) {
            // 5. 将点云数据进行融合
            fusion_cloud = perception_executor_->getHandCameraFusionCloud(object_clouds);
            if (!fusion_cloud.data.empty()) {
                return GetFusionCloudStatus::SUCCESS;
            }
        }
        RCLCPP_WARN(node_->get_logger(), "Attempt %d: only %zu views, skip fusion", attempt, object_clouds.size());
    }

    return GetFusionCloudStatus::FUSION_FAILED;
}

// ----- 根据相机获取到的物体中心位置，生成相机拍摄的三个位姿 -----
// 第一次尝试拍摄 Y 轴两侧，第二次尝试拍摄 X 轴两侧
void TaskExecutor::getObservationPoses(std::vector<geometry_msgs::msg::PoseStamped>& arm_poses, const geometry_msgs::msg::PoseStamped& object_pose, const tf2::Transform& camera_in_hand, int attempt) {
    const tf2::Vector3 object(object_pose.pose.position.x,
                              object_pose.pose.position.y,
                              object_pose.pose.position.z);

    const double tilt = (attempt == 0) ? 45.0 * M_PI / 180.0
                                       : 40.0 * M_PI / 180.0;      // 侧视角倾角

    const double cam_distance = (attempt == 0) ? 0.35 : 0.30;      // 相机到物体的距离
    const double yaw = (attempt == 0) ? 0.0 : M_PI / 2.0;
    const double s = std::sin(tilt);
    const double c = std::cos(tilt);

    // 相机外参常量
    const tf2::Vector3    p_cam_in_hand = camera_in_hand.getOrigin();
    const tf2::Matrix3x3  R_cam_in_hand = camera_in_hand.getBasis();

    // "手原点相对相机原点"的向量，表达在相机坐标系里
    const tf2::Vector3    hand_in_cam = R_cam_in_hand.transpose() * p_cam_in_hand;

    // panda_link8 <- panda_hand 的固定旋转
    const tf2::Matrix3x3  R_link8_in_hand(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -2.35619449));

    std::vector<tf2::Vector3> view_dirs;
    view_dirs.push_back(tf2::Vector3(0.0, 0.0, 1.0));
    view_dirs.push_back(tf2::Vector3( s * std::sin(yaw),  s * std::cos(yaw), c));
    view_dirs.push_back(tf2::Vector3(-s * std::sin(yaw), -s * std::cos(yaw), c));

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