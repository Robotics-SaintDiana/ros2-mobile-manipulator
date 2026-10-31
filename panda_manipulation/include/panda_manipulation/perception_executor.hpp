#ifndef PANDA_PERCEPTION__PERCEPTION_EXECUTOR_HPP_
#define PANDA_PERCEPTION__PERCEPTION_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <string>

class PerceptionExecutor
{
    public:
        // 构造函数
        explicit PerceptionExecutor(const rclcpp::Node::SharedPtr& node);

        // 对外暴露的接口
        // getPickCameraLatestProcessedCloud 操作 --- 返回抓取固定相机最新处理好的点云数据
        bool getPickCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& pick_camera_lasted_cloud);

        // getPlaceCameraLatestProcessedCloud 操作 --- 返回放置固定相机最新处理好的点云数据
        bool getPlaceCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& place_camera_lasted_cloud);

        // getHandCameraLatestProcessedCloud 操作 --- 返回手部固定相机最新处理好的点云数据
        bool getHandCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& hand_camera_lasted_cloud);

        // getPlatePosition 操作 --- 返回指定颜色托盘位置
        bool getPlatePosition(const std::string& plate_color, geometry_msgs::msg::Pose& place_pose);

    private:
        // 点云回调函数及相关辅助函数
        void pickCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void placeCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void handCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        
        void processAndPublishCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                    std::mutex& mutex,
                                    sensor_msgs::msg::PointCloud2& latest_cloud,
                                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

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
};

#endif