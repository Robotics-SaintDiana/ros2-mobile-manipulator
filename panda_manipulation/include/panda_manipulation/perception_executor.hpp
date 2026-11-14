#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <string>

class PerceptionExecutor
{
    public:
        // 构造函数
        explicit PerceptionExecutor(const rclcpp::Node::SharedPtr& node);

        // getObjectCenterPosition 函数中可能返回的状态码
        enum class GetObjectCenterStatus {SUCCESS, NO_DATA, NO_OBJECT, PCL_FAILED, GET_TF_FAILED};

        // 对外暴露的接口
        // 将手部相机在三个方向进行拍摄的点云数据进行融合
        sensor_msgs::msg::PointCloud2 getHandCameraFusionCloud(const std::vector<sensor_msgs::msg::PointCloud2>& camera_clouds);

        // 返回 pick 相机识别到的物体的中心位置
        bool getObjectCenterPosition(geometry_msgs::msg::PoseStamped& object_pose);
        GetObjectCenterStatus getObjectCenterPositionOnce(geometry_msgs::msg::PoseStamped& object_pose);
        
        // 返回指定颜色托盘位置
        bool getPlatePosition(const std::string& plate_color, geometry_msgs::msg::Pose& place_pose);

        // 返回相机最新处理好的点云数据
        bool getPickCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& pick_camera_lastest_cloud, const rclcpp::Time & min_stamp);
        bool getPlaceCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& panda_link0_lastest_cloud, const rclcpp::Time & min_stamp);
        bool getHandCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& panda_link0_lastest_cloud, const rclcpp::Time & min_stamp);

    private:
        // 读取本类的可调参数
        void loadParameters();

        // 点云回调函数
        void pickCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void placeCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void handCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        // 去除输入点云中的桌面
        sensor_msgs::msg::PointCloud2 processAndPublishCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

        // ----- ROS & PCL 句柄 -----
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pick_camera_point_cloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr place_camera_point_cloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr hand_camera_point_cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pick_camera_pcl_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr place_camera_pcl_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hand_camera_pcl_cloud_pub_;

        // ----- 最新点云缓存，以及相应的进程锁 -----
        sensor_msgs::msg::PointCloud2 pick_camera_latest_processed_cloud_;
        sensor_msgs::msg::PointCloud2 place_camera_latest_processed_cloud_;
        sensor_msgs::msg::PointCloud2 hand_camera_latest_processed_cloud_;
        mutable std::mutex pick_camera_cloud_mutex_;
        mutable std::mutex place_camera_cloud_mutex_;
        mutable std::mutex hand_camera_cloud_mutex_;

        // ----- 用来获取 TF 变换 -----
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ----- 可调参数 -----
        double hand_camera_min_range_   = 0.08;   // 手相机单帧：太近的点（手指/镜片）丢掉
        double hand_camera_max_range_   = 1.20;   // 手相机单帧：太远的点（背景/地面）丢掉
        double fusion_voxel_size_       = 0.005;  // 融合时的体素边长
        double workspace_half_extent_   = 0.30;   // 工作空间在 xy 方向的半边长
        double workspace_max_height_    = 0.50;   // 工作空间在桌面上方的高度
        double workspace_table_margin_  = 0.015;  // 工作空间在桌面下方保留的厚度（保证桌面点不被裁掉）
        int    fusion_min_points_       = 100;    // 融合后少于这个点数就认为失败
};