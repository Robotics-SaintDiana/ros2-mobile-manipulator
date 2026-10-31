#include "panda_manipulation/perception_executor.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 内联函数：将 RGB 颜色空间转换为 HSV 颜色空间
inline void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float& h, float& s, float& v) {
    // 1. 将 0~255 的整型 RGB 归一化到 0.0 ~ 1.0 的浮点数范围
    float rd = static_cast<float>(r) / 255.0f;
    float gd = static_cast<float>(g) / 255.0f;
    float bd = static_cast<float>(b) / 255.0f;

    // 2. 找出 RGB 三个分量中的最大值、最小值以及它们的极差
    float max_val = std::max({rd, gd, bd});
    float min_val = std::min({rd, gd, bd});
    float diff = max_val - min_val;

    // 3. 计算明度 V (Value)：V 直接等于 RGB 中的最大值
    v = max_val;

    // 4. 边界处理：如果最大值为 0，说明是纯黑色，此时饱和度和色相无意义，直接归零返回
    if (max_val == 0.0f) {
        s = 0.0f;
        h = 0.0f;
        return;
    }

    // 5. 计算饱和度 S (Saturation)：极差占最大值的比例
    s = diff / max_val;

    // 6. 边界处理：如果极差为 0，此时色相无意义，归零返回
    if (diff == 0.0f) {
        h = 0.0f;
        return;
    }

    // 7. 计算色相 H (Hue)：根据哪个分量是最大值，在 360 度色环上计算对应的夹角
    if (max_val == rd) {
        h = 60.0f * (gd - bd) / diff;
        if (h < 0.0f) h += 360.0f;
    } else if (max_val == gd) {
        h = 60.0f * (bd - rd) / diff + 120.0f;
    } else {
        h = 60.0f * (rd - gd) / diff + 240.0f;
    }
}

// 构造函数实现
PerceptionExecutor::PerceptionExecutor(const rclcpp::Node::SharedPtr& node) : node_(node) {
    // 1. 订阅抓取固定相机、放置固定相机和手部相机的点云话题
    pick_camera_point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pick_camera/depth/color/points",
        rclcpp::SensorDataQoS(),
        std::bind(&PerceptionExecutor::pickCameraPointCloudCallback, this, std::placeholders::_1)
    );

    place_camera_point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/place_camera/depth/color/points",
        rclcpp::SensorDataQoS(),
        std::bind(&PerceptionExecutor::placeCameraPointCloudCallback, this, std::placeholders::_1)
    );

    hand_camera_point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hand_camera/depth/color/points",
        rclcpp::SensorDataQoS(),
        std::bind(&PerceptionExecutor::handCameraPointCloudCallback, this, std::placeholders::_1)
    );

    // 2. 发布抓取固定相机、放置固定相机和手部相机的点云话题
    pick_camera_pcl_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/pick_camera_pcl_cloud_pub", 
        10
    );

    place_camera_pcl_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/place_camera_pcl_cloud_pub", 
        10
    );

    hand_camera_pcl_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/hand_camera_pcl_cloud_pub", 
        10
    );
}

// ----- 获取抓取固定相机的点云数据进行处理 -----
void PerceptionExecutor::pickCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 调用辅助函数，获取最新的点云数据
    processAndPublishCloud(msg, pick_camera_cloud_mutex_, pick_camera_latest_processed_cloud_, pick_camera_pcl_cloud_pub_);
}

// ----- 获取放置固定相机的点云数据进行处理 -----
void PerceptionExecutor::placeCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 调用辅助函数，获取最新的点云数据
    processAndPublishCloud(msg, place_camera_cloud_mutex_, place_camera_latest_processed_cloud_, place_camera_pcl_cloud_pub_);
}

// ----- 获取手部相机的点云数据进行处理 -----
void PerceptionExecutor::handCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 调用辅助函数，获取最新的点云数据
    processAndPublishCloud(msg, hand_camera_cloud_mutex_, hand_camera_latest_processed_cloud_, hand_camera_pcl_cloud_pub_);
}

// ----- 获取指定颜色托盘的放置位姿 -----
bool PerceptionExecutor::getPlatePosition(const std::string& plate_color, geometry_msgs::msg::Pose& place_pose) {
    // 1. 对传入的颜色参数进行合法性校验，其只能为 red、green 和 blue 之一
    static const std::unordered_set<std::string> VALID_COLORS = {"red", "green", "blue"};
    if (VALID_COLORS.find(plate_color) == VALID_COLORS.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid plate color: [%s]. Supported colors are: red, green, blue.", plate_color.c_str());
        return false;
    }

    // 2. 获取手部相机最新的点云数据
    sensor_msgs::msg::PointCloud2 plate_cloud;
    if (!getHandCameraLatestProcessedCloud(plate_cloud)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to get latest point cloud for plate detection.");
        return false;
    }

    // 3. 将 ROS 点云转换为 PCL 原生点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(plate_cloud, *pcl_cloud);

    if (pcl_cloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "Converted PCL point cloud is empty, skipping current processing.");
        return false;
    }

    // 4. 利用 HSV 空间提取指定颜色的托盘点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& point : *pcl_cloud) {
        // 4.1 有效点过滤，将数值为 NaN 或者 Inf 的点过滤掉
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 4.2 将 RGB 转为 HSV，并进行颜色匹配
        float point_h, point_s, point_v;
        rgbToHsv(point.r, point.g, point.b, point_h, point_s, point_v);

        bool is_match = false;
        if (plate_color == "red") {
            // 红色的 H 通常在 0~10 或 330~360 左右
            if (((point_h >= 0.0f && point_h <= 15.0f) || (point_h >= 330.0f && point_h <= 360.0f)) && point_s > 0.4f && point_v > 0.2f) {
                is_match = true;
            }
        } else if (plate_color == "green") {
            // 绿色的 H 通常在 90~150 左右
            if (point_h >= 90.0f && point_h <= 150.0f && point_s > 0.4f && point_v > 0.2f) {
                is_match = true;
            }
        } else if (plate_color == "blue") {
            // 蓝色的 H 通常在 200~260 左右
            if (point_h >= 200.0f && point_h <= 260.0f && point_s > 0.4f && point_v > 0.2f) {
                is_match = true;
            }
        }

        if (is_match) {
            color_filtered_cloud->push_back(point);
        }
    }

    // 4.3. 检查过滤后的点云数量是否足够
    if (color_filtered_cloud->size() < 50) {
        RCLCPP_WARN(node_->get_logger(), "Not enough points found for target color: [%s]", plate_color.c_str());
        return false;
    }

    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(color_filtered_cloud);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_OBB, max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f size_OBB;

    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    place_pose.position.x = position_OBB.x;
    place_pose.position.y = position_OBB.y;
    place_pose.position.z = position_OBB.z;

    // 设置四元数
    Eigen::Quaternionf q_eigen(rotational_matrix_OBB);
    place_pose.orientation.x = q_eigen.x();
    place_pose.orientation.y = q_eigen.y();
    place_pose.orientation.z = q_eigen.z();
    place_pose.orientation.w = q_eigen.w();

    return true;
}

// ----- 返回抓取固定相机最新处理好的点云数据 -----
bool PerceptionExecutor::getPickCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& pick_camera_lasted_cloud) {
    {
        std::lock_guard<std::mutex> lock(pick_camera_cloud_mutex_);

        // 如果没有最新的点云数据，报错返回
        if (pick_camera_latest_processed_cloud_.data.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the pick camera!");
            return false;
        }

        // 返回存储的最新点云数据
        pick_camera_lasted_cloud = pick_camera_latest_processed_cloud_;
    }
    return true;
}

// ----- 返回放置固定相机最新处理好的点云数据 -----
bool PerceptionExecutor::getPlaceCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& place_camera_lasted_cloud) {
    {
        std::lock_guard<std::mutex> lock(place_camera_cloud_mutex_);

        // 如果没有最新的点云数据，报错返回
        if (place_camera_latest_processed_cloud_.data.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the place camera!");
            return false;
        }

        // 返回存储的最新点云数据
        place_camera_lasted_cloud = place_camera_latest_processed_cloud_;
    }
    return true;
}

// ----- 返回放置固定相机最新处理好的点云数据 -----
bool PerceptionExecutor::getHandCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& hand_camera_lasted_cloud) {
    {
        std::lock_guard<std::mutex> lock(hand_camera_cloud_mutex_);

        // 如果没有最新的点云数据，报错返回
        if (hand_camera_latest_processed_cloud_.data.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the hand camera!");
            return false;
        }

        // 返回存储的最新点云数据
        hand_camera_lasted_cloud = hand_camera_latest_processed_cloud_;
    }
    return true;
}

// ----- 点云回调函数的辅助函数，基于 PCL 库对相机点云进行桌面去除，保存并发布最新的点云数据 -----
void PerceptionExecutor::processAndPublishCloud(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
            std::mutex& mutex,
            sensor_msgs::msg::PointCloud2& latest_cloud,
            const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    // 1. 将 ROS 的 sensor_msgs::msg::PointCloud2 转换为 PCL 原生点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "Converted PCL point cloud is empty, skipping current processing.");
        return;
    }

    // 2. 利用 RANSAC 拟合桌面方程，去除桌面部分
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());   // 存放平面参数的容器（Ax + By + Cz + D = 0 里的 A, B, C, D）
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                  // 存放属于桌面的点的索引的容器
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;                               // 实例化 PCL 的 RANSAC 平面分割对象
    seg.setOptimizeCoefficients(true);                                        // 优化拟合系数
    seg.setModelType(pcl::SACMODEL_PLANE);                                    // 设置寻找类型为平面
    seg.setMethodType(pcl::SAC_RANSAC);                                       // 设置求解类型为 RANSAC 算法
    seg.setMaxIterations(100);                                                // 设置最大迭代次数
    seg.setDistanceThreshold(0.005f);                                         // 如果某个点到拟合平面的距离小于 0.5cm，认为该点属于桌面  
    seg.setInputCloud(pcl_cloud);                                             // 传入原始点云数据
    seg.segment(*inliers, *coefficients);                                     // 执行分割

    bool has_table_plane = false;
    float a = 0, b = 0, c = 0, d = 0;

    if (!inliers->indices.empty() && coefficients->values.size() >= 4) {
        has_table_plane = true;
        a = coefficients->values[0];
        b = coefficients->values[1];
        c = coefficients->values[2];
        d = coefficients->values[3];
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to fit tabletop plane, falling back to basic valid point filtering.");
    }

    float norm_abc = std::sqrt(a * a + b * b + c * c);

    // 3. 直接提取桌面上方的所有物体点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& point : *pcl_cloud) {
        // 3.1 过滤无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || point.z <= 0.0f) {
            continue;
        }

        // 3.2 剔除桌面上的点
        if (has_table_plane && norm_abc > 1e-5f) {
            float distance = std::abs(a * point.x + b * point.y + c * point.z + d) / norm_abc;
            if (distance < 0.003f) {
                continue;
            }
        }

        // 将所有合法的非桌面点加入目标点云
        object_cloud->push_back(point);
    }

    // 4. 检查点云数量
    if (object_cloud->size() < 50) {
        RCLCPP_WARN(node_->get_logger(), "当前场景中未提取到足够的物体 3D 点云！");
        return;
    }

    // 5. 转换为 ROS 消息
    sensor_msgs::msg::PointCloud2 output_ros_cloud;
    pcl::toROSMsg(*object_cloud, output_ros_cloud);
    output_ros_cloud.header = msg->header; 

    // 6. 存储最新的点云信息，并将其发布出去
    {
        std::lock_guard<std::mutex> lock(mutex);
        latest_cloud = output_ros_cloud;
    }
    publisher->publish(output_ros_cloud);
}