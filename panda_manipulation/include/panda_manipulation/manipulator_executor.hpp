#ifndef PANDA_MANIPULATION__MANIPULATION_EXECUTOR_HPP_
#define PANDA_MANIPULATION__MANIPULATION_EXECUTOR_HPP_

#include <vector>
#include <mutex>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ManipulatorExecutor
{
    public:
        // 构造函数
        explicit ManipulatorExecutor(const rclcpp::Node::SharedPtr& node);

        // 对外暴露接口
        // 从物料台上抓取物体
        bool pickExecutor(const geometry_msgs::msg::PoseStamped& pick_pose);

        // 将物体放置在放置台上
        bool placeExecutor(const geometry_msgs::msg::PoseStamped& place_pose);

        // 控制机械臂运动至指定位姿处
        bool armMoveExecutor(const geometry_msgs::msg::PoseStamped& arm_pose);

        // 在 Pick 和 Place 流程中向 Moveit 的规划场景中添加或者删除物体，防止机械臂与其发生碰撞
        bool preparePickCollisionScene(double center_x, double center_y);
        bool preparePlaceCollisionScene(double center_x, double center_y, double center_z);
        bool removePickCollisionScene();
        bool removePlaceCollisionScene();

        // ----- 安全获取两个坐标系之间的 TF 变换 -----
        bool getTransform(const std::string & target_frame, const std::string & source_frame, tf2::Transform & transform_out);

        // ----- 获取当前机械臂的所有关节角度状态 -----
        std::vector<double> getCurrentJointValues();

        bool moveHome();

    private:
        // ----- 机械臂运动辅助方法 -----
        bool moveArmToPose(const geometry_msgs::msg::PoseStamped & target_pose, bool use_cartesian = false);
        bool moveArmToJoints(const std::vector<double>& joint_positions);
        bool controlGripper(bool open);
        bool closeGripperStepwise();
        bool descendArmStepwise(geometry_msgs::msg::PoseStamped & initial_pose);

        // ----- 机械臂任务执行合法性判断 -----
        bool isRobotReady();
        bool validateAndTransformPose(const geometry_msgs::msg::PoseStamped & input_pose_stamped, geometry_msgs::msg::PoseStamped & output_pose_stamped, const std::string task_stage);
        bool isPoseValidate(const geometry_msgs::msg::PoseStamped & target_pose);

        // ----- 抓取物体位姿信息回调函数，以及获取抓取物体最新位姿数据函数-----
        void objectPoseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        bool getObjectLatestPose(geometry_msgs::msg::Point & object_pose);

        // ----- 夹爪位姿信息回调函数，以及获取抓取物体最新位姿数据函数-----
        void fingerPoseCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        bool getFingerLatestPose(double & finger_pose);

        // ----- 向 Moveit 的规划场景中添加或者删除物体 -----
        bool addBoxToScene(const std::string & box_id, geometry_msgs::msg::Pose & box_pose, double box_x, double box_y, double box_z);
        bool removeObjectFromScene(const std::string & object_id);

        // ----- ROS & MoveIt 句柄 -----
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr object_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr finger_joint_sub_;

        // ----- 用来获取 TF 变换 -----
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ----- 预先设定好的关节角度 -----
        std::vector<double> start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -125.0 * M_PI / 180.0};
        std::vector<double> home_joints  = {0.0, 0.0, 0.0, -90.0 * M_PI / 180.0, 0.0, 92.0 * M_PI / 180.0, 50.0 * M_PI / 180.0};
        
        // ----- 抓取物体的 Z 方向位姿，以及相应的进程锁 -----
        struct ObjectPose { rclcpp::Time objectStamp; geometry_msgs::msg::Point objectPose; };
        ObjectPose latest_object_pose_;
        bool has_object_pose_ = false;
        mutable std::mutex object_pose_mutex_;

        // ----- 获取夹爪关节位置，以及相应的进程锁 -----
        struct FingerPose { rclcpp::Time fingerStamp; double fingerPose; };
        FingerPose latest_finger_pose_;
        bool has_finger_joint_ = false;
        mutable std::mutex finger_pose_mutex_;
        
        // ----- 桌面碰撞物体的参数（基于 panda_link0 坐标系） -----
        const double table_top_z_ = 0.035;         // 桌面上表面在 panda_link0 下的高度
        const double table_thickness_ = 0.10;
        const double plate_thickness_ = 0.04;
        const double pick_table_size_x_ = 0.60;    
        const double pick_table_size_y_ = 0.60; 
        const double place_table_size_x_ = 0.80;
        const double place_table_size_y_ = 0.80;     
        const double plate_size_ = 0.25;  
}; 

#endif