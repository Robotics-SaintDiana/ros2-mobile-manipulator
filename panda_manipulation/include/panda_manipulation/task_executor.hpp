#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "panda_manipulation/manipulator_executor.hpp"
#include "panda_manipulation/perception_executor.hpp"

class TaskExecutor
{
    public:
        // 构造函数
        TaskExecutor(rclcpp::Node::SharedPtr node, std::shared_ptr<ManipulatorExecutor> manipulator, std::shared_ptr<PerceptionExecutor> perception);

        // captureFusionCloud 可能返回的状态码
        enum class GetFusionCloudStatus {SUCCESS, GET_TF_FAILED, FUSION_FAILED};

        // 获取目标物体的多角度融合点云
        GetFusionCloudStatus captureFusionCloud(geometry_msgs::msg::PoseStamped& object_pose, sensor_msgs::msg::PointCloud2& fusion_cloud);

    private:
        // 根据相机获取到的物体中心位置，生成相机拍摄的三个位姿
        void getObservationPoses(std::vector<geometry_msgs::msg::PoseStamped>& arm_poses, const geometry_msgs::msg::PoseStamped& object_pose, const tf2::Transform& camera_in_hand, int attempt);
        
        // ----- ROS & MoveIt 句柄 -----
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<ManipulatorExecutor> manipulator_executor_;
        std::shared_ptr<PerceptionExecutor> perception_executor_;
};


