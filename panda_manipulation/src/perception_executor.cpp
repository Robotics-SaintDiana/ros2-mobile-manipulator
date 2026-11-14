#include "panda_manipulation/perception_executor.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

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

    // 3. 初始化 TF2 监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. 读取默认参数
    loadParameters();
}

// ----- 将手部相机三次拍摄到的点云数据进行融合 -----
sensor_msgs::msg::PointCloud2 PerceptionExecutor::getHandCameraFusionCloud(const std::vector<sensor_msgs::msg::PointCloud2>& camera_clouds) {
    // 1. 对传入的点云数据进行有效性判断，不能为空；为空则返回空点云
    if (camera_clouds.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Received empty cloud list for fusion!");
        return sensor_msgs::msg::PointCloud2();
    }

    // 2. 创建用于存放合并后原始点云的容器
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusion_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 3. 循环遍历所有点云，直接转为 PCL 格式并累加
    for (const auto& camera_cloud : camera_clouds) {
        // 如果传入的点云存在为空的情况，该点云不进行任何处理
        if (camera_cloud.data.empty()) continue;

        // 如果传入的点云数据的 frame_id 错误，跳过此次操作
        if (camera_cloud.header.frame_id != "panda_link0") {
            RCLCPP_ERROR(node_->get_logger(), "Cloud frame mismatch! Expected panda_link0, got: %s", camera_cloud.header.frame_id.c_str());
            continue;
        }

        // 将 ROS 消息转为 PCL 格式
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(camera_cloud, *transformed_cloud); 

        // 累加到全局容器中
         *fusion_cloud += *transformed_cloud;
    }

    // 防止传入的所有点云全为空
    if (fusion_cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "All input clouds are empty after conversion!");
        return sensor_msgs::msg::PointCloud2();
    }

    // 4. 进行体素网格滤波降采样，清理重叠区域冗余点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampling_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setLeafSize(static_cast<float>(fusion_voxel_size_),
                      static_cast<float>(fusion_voxel_size_),
                      static_cast<float>(fusion_voxel_size_));
    voxel.setInputCloud(fusion_cloud);
    voxel.filter(*downsampling_cloud);

    if (downsampling_cloud->size() < 50) {
        RCLCPP_ERROR(node_->get_logger(), "Too few points after voxel filtering: %zu", downsampling_cloud->size());
        return sensor_msgs::msg::PointCloud2();
    }

    // 5. 从降采样的桌面中识别出桌面，并以此为基准进行工作空间裁减，保留桌面及上方的物体点云(保留桌子用于在 GraspNet 中进行碰撞检测)
    // 5.1 利用 RANSAC 算法拟合桌面平面
    float table_z = 0.0f;
    bool has_table_plane = false;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());   // 存放平面参数的容器（Ax + By + Cz + D = 0 里的 A, B, C, D）
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                  // 存放属于桌面的点的索引的容器
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;                               // 实例化 PCL 的 RANSAC 平面分割对象
    seg.setOptimizeCoefficients(true);                                        // 优化拟合系数
    seg.setModelType(pcl::SACMODEL_PLANE);                                    // 设置寻找类型为平面
    seg.setMethodType(pcl::SAC_RANSAC);                                       // 设置求解类型为 RANSAC 算法
    seg.setMaxIterations(100);                                                // 设置最大迭代次数
    seg.setDistanceThreshold(0.015f);                                         // 如果某个点到拟合平面的距离小于 1cm，认为该点属于桌面  
    seg.setInputCloud(downsampling_cloud);                                    // 传入原始点云数据
    seg.segment(*inliers, *coefficients);                                     // 执行分割

    // 要求桌子的点云数量最少要为 50，点云数量多的话，要求桌子的点云数量要达到总点数的 10% 以上
    const size_t min_inliers = std::max<size_t>(50, downsampling_cloud->size() / 10);

    // 确认点云数量足够，且 RANSAC 算法返回的参数合理
    if (inliers->indices.size() >= min_inliers && coefficients->values.size() >= 4) {
        // 拟合桌面的法向量进行校验
        const float a = coefficients->values[0];
        const float b = coefficients->values[1];
        const float c = coefficients->values[2];
        const float norm = std::sqrt(a * a + b * b + c * c);

        // 桌子一定是水平的，故法向量应几乎平行于 Z 轴，如果其 Z 轴分量过小，代表有可能识别到了墙面或者物体侧面
        if (norm > 1e-5f && std::abs(c) / norm > 0.8f) {
            // 获取贴合桌面点的 Z 坐标，计算其中位数作为桌面高度
            std::vector<float> inlier_zs;
            inlier_zs.reserve(inliers->indices.size());
            for (int index : inliers->indices) {
                inlier_zs.push_back((*downsampling_cloud)[index].z);
            }
            std::nth_element(inlier_zs.begin(),
                                inlier_zs.begin() + inlier_zs.size() / 2,
                                inlier_zs.end());
            table_z = inlier_zs[inlier_zs.size() / 2];
            has_table_plane = true;
        }
    }

    // 如果没有在点云中识别到桌面，为了避免裁切掉物体的点云，用最低 5% 分位的点云高度作为默认的桌面高度
    if (!has_table_plane) {
        // 获取所有降采样完后的点云的 Z 方向高度
        std::vector<float> all_zs;
        all_zs.reserve(downsampling_cloud->size());
        for (const auto& point : *downsampling_cloud) {
            all_zs.push_back(point.z);
        }

        // 计算最低 5% 分位的位置索引，并将该数据作为桌面高度
        const size_t index = all_zs.size() * 30 / 100;
        std::nth_element(all_zs.begin(), all_zs.begin() + index, all_zs.end());
        table_z = all_zs[index];

        RCLCPP_WARN(node_->get_logger(), "Table plane not found, fallback table_z = %.3f m", table_z);
    }

    // 5.2 仅统计位于桌面上方 1cm 以上的点，计算其几何中心作为桌面物体的大致中心
    double center_x = 0.0;
    double center_y = 0.0;
    size_t object_point_count = 0;

    for (const auto& point : *downsampling_cloud) {
        if (point.z > table_z + 0.01f) {
            center_x += point.x;
            center_y += point.y;
            ++object_point_count;
        }
    }
    const bool has_object = object_point_count >= 30;
    if (has_object) {
        center_x /= static_cast<double>(object_point_count);
        center_y /= static_cast<double>(object_point_count);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Too few points above the table, skip xy workspace cropping.");
    }

    // 5.3 基于上述计算的数据进行工作空间裁减，保留桌面及上方的物体点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr workspace_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    workspace_cloud->reserve(downsampling_cloud->size());

    // Z 轴下界多保留一点，防止贴在桌面表面的点被误切掉
    const float z_min = table_z - static_cast<float>(workspace_table_margin_);
    const float z_max = table_z + static_cast<float>(workspace_max_height_);
    const double half_extent = workspace_half_extent_;

    // 筛选点云
    for (const auto& point : *downsampling_cloud) {
        // 过滤无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 高度方向过滤
        if (point.z < z_min || point.z > z_max) {
            continue;
        }

        // 水平方向过滤
        if (has_object &&
            (std::abs(point.x - center_x) > half_extent || std::abs(point.y - center_y) > half_extent)) {
            continue;
        }

        workspace_cloud->push_back(point);
    }

    // 如果最后的点云总数少于设定值，可能点云中压根没东西，不再传递给 GraspNet 推理
    if (static_cast<int>(workspace_cloud->size()) < fusion_min_points_) {
        RCLCPP_ERROR(node_->get_logger(), "Workspace cloud too small: %zu points (min %d)",
                     workspace_cloud->size(), fusion_min_points_);
        return sensor_msgs::msg::PointCloud2();
    }

    // 6. 转换回 sensor_msgs 返回
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*workspace_cloud, output_cloud);
    output_cloud.header.stamp = node_->now();
    output_cloud.header.frame_id = "panda_link0";

    return output_cloud;
}

// ----- 获取 pick 相机识别到的物体的中心位置(临时) -----
bool PerceptionExecutor::getObjectCenterPosition(geometry_msgs::msg::PoseStamped& object_pose) {
    // 这里常见的问题有：无法获取相机最新的点云数据；去除桌面后点云为空；以及 TF 变换获取失败
    // 以上三个问题均采用失败重试机制，最多尝试三次
    rclcpp::Time start_time = node_->now();
    for (int attempt = 0; attempt < 3; attempt++) {
        // 1. 先获取 pick 相机最新的点云数据；如果没有获取到最新的点云数据，重新尝试
        sensor_msgs::msg::PointCloud2 pick_camera_latest_cloud;
        if (!getPickCameraLatestProcessedCloud(pick_camera_latest_cloud, start_time)) {
            RCLCPP_ERROR(node_->get_logger(), "Can not get latest pick camera cloud!");
            continue;
        }

        // 更新时间戳，用于下一次重试
        start_time = node_->now();

        // 2. 去除点云数据中的桌面部分，去除后为空点云的话可能是点云存在问题，重试
        auto pick_camera_lasted_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(pick_camera_latest_cloud);
        sensor_msgs::msg::PointCloud2 filiter_cloud = processAndPublishCloud(pick_camera_lasted_cloud_ptr);
        if (filiter_cloud.data.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Pick camera filter cloud is empty!");
            continue;
        }

        // 3. 计算物体的中心位置作为物体的大致位置
        // 3.1 首先先将过滤后的点云数据转为 PCL 原生点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(filiter_cloud, *pcl_cloud);
        if (pcl_cloud->empty()) {
            RCLCPP_ERROR(node_->get_logger(), "PCL cloud is empty after conversion!");
            continue;
        }

        // 3.2 计算三维平均坐标
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pcl_cloud, centroid);

        // 3.3 把物体的中心位置变换到 panda_link0 坐标系下
        const std::string cloud_frame_id = pick_camera_latest_cloud.header.frame_id;
        geometry_msgs::msg::TransformStamped transform_stamped;
        bool is_transform_found = false;

        // 先尝试获取点云时间戳所对应的 tf 变换
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "panda_link0", cloud_frame_id,
                rclcpp::Time(pick_camera_latest_cloud.header.stamp, node_->get_clock()->get_clock_type()));
            is_transform_found = true;
        } catch (const tf2::TransformException &) {}

        // 如果查询不到，再用当前时刻的 tf 变换，如果当前时刻 TF 变换也查询不到则重试
        if (!is_transform_found) {
            try {
                transform_stamped = tf_buffer_->lookupTransform("panda_link0", cloud_frame_id, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get TF from [%s] to [panda_link0]: %s", cloud_frame_id.c_str(), ex.what());
                continue;
            }
        }

        // 转换至 panda_link0 坐标系下
        tf2::Transform tf_link0_to_camera;
        tf2::fromMsg(transform_stamped.transform, tf_link0_to_camera);
        const tf2::Vector3 point_in_link0 = tf_link0_to_camera * tf2::Vector3(centroid[0], centroid[1], centroid[2]);

        // 3.4 返回物体的中心位置
        object_pose.header.frame_id = "panda_link0";
        object_pose.header.stamp = pick_camera_latest_cloud.header.stamp;

        object_pose.pose.position.x = point_in_link0.x();
        object_pose.pose.position.y = point_in_link0.y();
        object_pose.pose.position.z = point_in_link0.z();

        // 默认四元数为数值向下(因为此处仅仅是配合抓取物体时识别大致中心位置，不需要很精确)
        object_pose.pose.orientation.x = 0.0;
        object_pose.pose.orientation.y = 0.0;
        object_pose.pose.orientation.z = 0.0;
        object_pose.pose.orientation.w = 1.0;

        return true;    
    }

    // 重试三次后仍然错误则认为失败
    RCLCPP_ERROR(node_->get_logger(), "getObjectCenterPosition failed after 3 attempts");
    return false;
}

// ----- 获取 pick 相机识别到的物体的中心位置 -----
PerceptionExecutor::GetObjectCenterStatus PerceptionExecutor::getObjectCenterPositionOnce(geometry_msgs::msg::PoseStamped& object_pose) {
    // 1. 先获取 pick 相机最新的点云数据
    sensor_msgs::msg::PointCloud2 pick_camera_latest_cloud;
    rclcpp::Time start_time = node_->now();
    if (!getPickCameraLatestProcessedCloud(pick_camera_latest_cloud, start_time)) {
        return GetObjectCenterStatus::NO_DATA;
    }

    // 2. 去除点云数据中的桌面部分
    auto pick_camera_lasted_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(pick_camera_latest_cloud);
    sensor_msgs::msg::PointCloud2 filiter_cloud = processAndPublishCloud(pick_camera_lasted_cloud_ptr);
    if (filiter_cloud.data.empty()) {
        return GetObjectCenterStatus::NO_OBJECT;
    }

    // 3. 计算物体的中心位置作为物体的大致位置
    // 3.1 首先先将过滤后的点云数据转为 PCL 原生点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(filiter_cloud, *pcl_cloud);
    if (pcl_cloud->empty()) {
        return GetObjectCenterStatus::PCL_FAILED;
    }

    // 3.2 计算三维平均坐标
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);

    // 3.3 把物体的中心位置变换到 panda_link0 坐标系下
    const std::string cloud_frame_id = pick_camera_latest_cloud.header.frame_id;
    geometry_msgs::msg::TransformStamped transform_stamped;
    bool is_transform_found = false;

    // 先尝试获取点云时间戳所对应的 tf 变换
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            "panda_link0", cloud_frame_id,
            rclcpp::Time(pick_camera_latest_cloud.header.stamp, node_->get_clock()->get_clock_type()));
        is_transform_found = true;
    } catch (const tf2::TransformException &) {}

    // 如果查询不到，再用当前时刻的 tf 变换
    if (!is_transform_found) {
        try {
            transform_stamped = tf_buffer_->lookupTransform("panda_link0", cloud_frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return GetObjectCenterStatus::GET_TF_FAILED;
        }
    }

    // 转换至 panda_link0 坐标系下
    tf2::Transform tf_link0_to_camera;
    tf2::fromMsg(transform_stamped.transform, tf_link0_to_camera);
    const tf2::Vector3 point_in_link0 = tf_link0_to_camera * tf2::Vector3(centroid[0], centroid[1], centroid[2]);

    // 3.4 返回物体的中心位置
    object_pose.header.frame_id = "panda_link0";
    object_pose.header.stamp = pick_camera_latest_cloud.header.stamp;

    object_pose.pose.position.x = point_in_link0.x();
    object_pose.pose.position.y = point_in_link0.y();
    object_pose.pose.position.z = point_in_link0.z();

    // 默认四元数为数值向下(因为此处仅仅是配合抓取物体时识别大致中心位置，不需要很精确)
    object_pose.pose.orientation.x = 0.0;
    object_pose.pose.orientation.y = 0.0;
    object_pose.pose.orientation.z = 0.0;
    object_pose.pose.orientation.w = 1.0;

    return GetObjectCenterStatus::SUCCESS;    
}

// ----- 获取指定颜色托盘的放置位姿 -----
bool PerceptionExecutor::getPlatePosition(const std::string& plate_color, geometry_msgs::msg::Pose& place_pose) {
    // 1. 对传入的颜色参数进行合法性校验，其只能为 red、green 和 blue 之一
    static const std::unordered_set<std::string> VALID_COLORS = {"red", "green", "blue"};
    if (VALID_COLORS.find(plate_color) == VALID_COLORS.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid plate color: [%s]. Supported colors are: red, green, blue.", plate_color.c_str());
        return false;
    }
    
    // 这里常见的问题有：无法获取相机最新的点云数据；以及过滤后的点云数量不足
    // 以上两个问题均采用失败重试机制，最多尝试三次
    rclcpp::Time start_time = node_->now();

    for (int attempt = 0; attempt < 3; attempt++) { 
        // 2. 获取放置台固定相机最新的点云数据；如果没有获取到最新的点云数据，重新尝试
        sensor_msgs::msg::PointCloud2 plate_cloud;
        if (!getPlaceCameraLatestProcessedCloud(plate_cloud, start_time)) {
            RCLCPP_ERROR(node_->get_logger(), "Can not get latest place camera cloud!");
            continue;
        }

        // 更新时间戳，用于下一次重试
        start_time = node_->now();

        // 3. 将 ROS 点云转换为 PCL 原生点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(plate_cloud, *pcl_cloud);

        if (pcl_cloud->empty()) {
            RCLCPP_WARN(node_->get_logger(), "Converted PCL point cloud is empty, skipping current processing.");
            continue;
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
            RCLCPP_ERROR(node_->get_logger(), "Not enough points found for target color: [%s]", plate_color.c_str());
            continue;
        }

        // 4.4 计算目标颜色点云的质心，作为托盘中心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*color_filtered_cloud, centroid);
        place_pose.position.x = centroid[0];
        place_pose.position.y = centroid[1];
        place_pose.position.z = centroid[2];

        // 4.5 考虑到托盘是平整的，直接将放置姿态固定为竖直向下
        place_pose.orientation.x = 1.0;
        place_pose.orientation.y = 0.0;
        place_pose.orientation.z = 0.0;
        place_pose.orientation.w = 0.0;

        return true;
    }

    // 重试三次后仍然错误则认为失败
    RCLCPP_ERROR(node_->get_logger(), "getPlatePosition failed after 3 attempts");
    return false;
}

// ----- 返回抓取固定相机最新处理好的点云数据 -----
bool PerceptionExecutor::getPickCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& pick_camera_lastest_cloud, const rclcpp::Time & min_stamp) {
    // 处理思路与手部相机相同，设置最长等待时间，在此时间内等待最新的点云数据
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    bool has_latest_cloud = false;
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(pick_camera_cloud_mutex_);

            // 尝试获取最新的点云数据，要求数据不为空且点云时间戳大于函数调用时刻
            if (!pick_camera_latest_processed_cloud_.data.empty() &&
                rclcpp::Time(pick_camera_latest_processed_cloud_.header.stamp) > min_stamp)
            {
                pick_camera_lastest_cloud = pick_camera_latest_processed_cloud_;
                has_latest_cloud = true;
                break;
            }
        }
         std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    if (!has_latest_cloud) {
        RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the pick camera!");
        return false;
    }
    
    return true;
}

// ----- 返回放置固定相机最新处理好的点云数据，并转换到 panda_link0 坐标系下 -----
bool PerceptionExecutor::getPlaceCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& panda_link0_lastest_cloud, const rclcpp::Time & min_stamp) {
    sensor_msgs::msg::PointCloud2 place_camera_lasted_cloud;

    // 处理思路与手部相机相同，设置最长等待时间，在此时间内等待最新的点云数据
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    bool has_latest_cloud = false;
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(place_camera_cloud_mutex_);

            // 尝试获取最新的点云数据，要求数据不为空且点云时间戳大于函数调用时刻
            if (!place_camera_latest_processed_cloud_.data.empty() && 
                rclcpp::Time(place_camera_latest_processed_cloud_.header.stamp) > min_stamp) 
            {
                place_camera_lasted_cloud = place_camera_latest_processed_cloud_;
                has_latest_cloud = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!has_latest_cloud) {
        RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the place camera!");
        return false;
    }

    // 此处同样将获取到的相机点云数据转换到 panda_link0 下，方式与 getHandCameraLatestProcessedCloud 中一致
    const std::string cloud_frame_id = place_camera_lasted_cloud.header.frame_id;
    if (cloud_frame_id.empty() || cloud_frame_id != "place_camera_depth_optical_frame") {
        RCLCPP_ERROR(node_->get_logger(), "Invalid hand camera cloud frame_id: '%s'", cloud_frame_id.c_str());
        return false;
    }

    // 获取两者之间的 tf 变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    bool is_transform_found = false;

    try {
        transform_stamped = tf_buffer_->lookupTransform(
            "panda_link0", cloud_frame_id,
            rclcpp::Time(place_camera_lasted_cloud.header.stamp, node_->get_clock()->get_clock_type()));

        is_transform_found = true;    
    } catch (const tf2::TransformException &) {}

    if (!is_transform_found) {
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "panda_link0",
                cloud_frame_id,
                tf2::TimePointZero
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get latest TF: %s", ex.what());
            return false;
        }
    }

    // 将 ROS 消息转为 PCL 格式
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(place_camera_lasted_cloud, *pcl_cloud);
    if (pcl_cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Converted hand camera cloud is empty!");
        return false;
    }

    // 再转换到 panda_link0 坐标系下
    Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_matrix);
    if (transformed_cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Transformed hand camera cloud is empty!");
        return false;
    }

    // 最后将 PCL 点云转回 ROS
    pcl::toROSMsg(*transformed_cloud, panda_link0_lastest_cloud);
    panda_link0_lastest_cloud.header.frame_id = "panda_link0";
    panda_link0_lastest_cloud.header.stamp = place_camera_lasted_cloud.header.stamp;
    
    return true;
}

// ----- 返回手部相机最新处理好的点云数据，并转换到 panda_link0 坐标系下 -----
bool PerceptionExecutor::getHandCameraLatestProcessedCloud(sensor_msgs::msg::PointCloud2& panda_link0_lastest_cloud, const rclcpp::Time & min_stamp) {
    sensor_msgs::msg::PointCloud2 hand_camera_lasted_cloud;

    // 手部相机的点云数据后续要进行融合及推理，所以必须用最新时刻的点云
    // 这里的主要处理思路是设置一个最长等待时间，在此时间内等待最新的点云数据，没有最新数据则认为失败
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    bool has_latest_cloud = false;
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(hand_camera_cloud_mutex_);

            // 尝试获取最新的点云数据，要求数据不为空且点云时间戳大于函数调用时刻
            if (!hand_camera_latest_processed_cloud_.data.empty() && 
                rclcpp::Time(hand_camera_latest_processed_cloud_.header.stamp) > min_stamp) 
            {
                hand_camera_lasted_cloud = hand_camera_latest_processed_cloud_;
                has_latest_cloud = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!has_latest_cloud) {
        RCLCPP_WARN(node_->get_logger(), "No latest processed point cloud data available from the hand camera!");
        return false;
    }

    // 获取到最新的点云后，直接在此处将获取到的手部相机点云数据转换到 panda_link0 下，首先获取两者之间的 tf 变换
    // 先按点云自身时间戳查，该方法最为精确；如果查询不到则采用当前时刻的 tf 变换替代，因为机械臂保持静止，所以两者基本上等价
    // 先检查查询到的点云数据的 frame_id 是否合法，防止后续查询 tf 变换出错
    const std::string cloud_frame_id =  hand_camera_lasted_cloud.header.frame_id;
    if (cloud_frame_id.empty() || cloud_frame_id != "hand_camera_depth_optical_frame") {
        RCLCPP_ERROR(node_->get_logger(), "Invalid hand camera cloud frame_id: '%s'", cloud_frame_id.c_str());
        return false;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    bool is_transform_found = false;

    // 先按点云自身时间戳查
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            "panda_link0", cloud_frame_id,
            rclcpp::Time(hand_camera_lasted_cloud.header.stamp, node_->get_clock()->get_clock_type()));

        // 标志位置位
        is_transform_found = true;    
    } catch (const tf2::TransformException &) {}

    // 未查询到再采用当前时刻的 tf 变换
    if (!is_transform_found) {
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "panda_link0",
                cloud_frame_id,
                tf2::TimePointZero
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get latest TF: %s", ex.what());
            return false;
        }
    }

    // 将 ROS 消息转为 PCL 格式
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(hand_camera_lasted_cloud, *pcl_cloud);
    if (pcl_cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Converted hand camera cloud is empty!");
        return false;
    }

    // 再转换到 panda_link0 坐标系下
    Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_matrix);
    if (transformed_cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Transformed hand camera cloud is empty!");
        return false;
    }

    // 最后将 PCL 点云转回 ROS
    pcl::toROSMsg(*transformed_cloud, panda_link0_lastest_cloud);
    panda_link0_lastest_cloud.header.frame_id = "panda_link0";
    panda_link0_lastest_cloud.header.stamp = hand_camera_lasted_cloud.header.stamp;

    return true;
}

// ----- 读取可调参数 -----
void PerceptionExecutor::loadParameters() {
    // 检查节点中是否含有指定的参数，存在则获得其当前值，不存在则设置为默认值
    auto get_double = [this](const std::string& name, double default_value) -> double {
        if (!node_->has_parameter(name)) {
            return node_->declare_parameter<double>(name, default_value);
        }
        return node_->get_parameter(name).as_double();
    };

    hand_camera_min_range_  = get_double("hand_camera_min_range", 0.08);
    hand_camera_max_range_  = get_double("hand_camera_max_range", 1.20);
    fusion_voxel_size_      = get_double("fusion_voxel_size", 0.002);
    workspace_half_extent_  = get_double("workspace_half_extent", 0.30);
    workspace_max_height_   = get_double("workspace_max_height", 0.50);
    workspace_table_margin_ = get_double("workspace_table_margin", 0.03);

    if (!node_->has_parameter("fusion_min_points")) {
        fusion_min_points_ = node_->declare_parameter<int>("fusion_min_points", 100);
    } else {
        fusion_min_points_ = static_cast<int>(node_->get_parameter("fusion_min_points").as_int());
    }
}

// ----- 获取抓取固定相机的点云数据进行处理 -----
void PerceptionExecutor::pickCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 调用辅助函数，获取最新的点云数据
    sensor_msgs::msg::PointCloud2 output_ros_cloud = processAndPublishCloud(msg);

    // 存储最新的点云信息，并将其发布出去
    {
        std::lock_guard<std::mutex> lock(pick_camera_cloud_mutex_);
        pick_camera_latest_processed_cloud_ = output_ros_cloud;
    }
    pick_camera_pcl_cloud_pub_->publish(output_ros_cloud);
}

// ----- 获取放置固定相机的点云数据进行处理 -----
void PerceptionExecutor::placeCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 调用辅助函数，获取最新的点云数据
    sensor_msgs::msg::PointCloud2 output_ros_cloud = processAndPublishCloud(msg);

    // 存储最新的点云信息，并将其发布出去
    {
        std::lock_guard<std::mutex> lock(place_camera_cloud_mutex_);
        place_camera_latest_processed_cloud_ = output_ros_cloud;
    }
    place_camera_pcl_cloud_pub_->publish(output_ros_cloud);
}

// ----- 获取手部相机的点云数据进行处理 -----
void PerceptionExecutor::handCameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // 存储最新的点云信息，并将其发布出去
    {
        std::lock_guard<std::mutex> lock(hand_camera_cloud_mutex_);
        hand_camera_latest_processed_cloud_ = *msg;
    }
    hand_camera_pcl_cloud_pub_->publish(*msg);
}

// ----- 基于 PCL 库对相机点云进行桌面去除 -----
sensor_msgs::msg::PointCloud2 PerceptionExecutor::processAndPublishCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    // 1. 将 ROS 的 sensor_msgs::msg::PointCloud2 转换为 PCL 原生点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "Converted PCL point cloud is empty, skipping current processing.");
        return sensor_msgs::msg::PointCloud2();
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
        return sensor_msgs::msg::PointCloud2();
    }

    // 5. 转换为 ROS 消息
    sensor_msgs::msg::PointCloud2 output_ros_cloud;
    pcl::toROSMsg(*object_cloud, output_ros_cloud);
    output_ros_cloud.header = msg->header; 

    return output_ros_cloud;
}