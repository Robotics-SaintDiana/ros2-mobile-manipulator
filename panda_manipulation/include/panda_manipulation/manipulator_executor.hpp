#ifndef PANDA_MANIPULATION__MANIPULATION_EXECUTOR_HPP_
#define PANDA_MANIPULATION__MANIPULATION_EXECUTOR_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
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

    private:
        // ----- 机械臂运动辅助方法 -----
        bool moveArmToPose(const geometry_msgs::msg::PoseStamped & target_pose, bool use_cartesian = false);
        bool moveArmToJoints(const std::vector<double>& joint_positions);
        bool controlGripper(bool open);

        // ----- 机械臂任务执行合法性判断 -----
        bool isRobotReady();
        bool validateAndTransformPose(const geometry_msgs::msg::PoseStamped & input_pose_stamped, geometry_msgs::msg::PoseStamped & output_pose_stamped, const std::string task_stage);
        bool isPoseValidate(const geometry_msgs::msg::PoseStamped& target_pose);

        // ----- 安全获取两个坐标系之间的 TF 变换 -----
        bool getTransform(const std::string & target_frame, const std::string & source_frame, tf2::Transform & transform_out);

        // ----- ROS & MoveIt 句柄 -----
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;

        // ----- 用来获取 TF 变换 -----
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ----- 预先设定好的关节角度 -----
        std::vector<double> start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -125.0 * M_PI / 180.0};
        std::vector<double> home_joints  = {0.0, 0.0, 0.0, -90.0 * M_PI / 180.0, 0.0, 92.0 * M_PI / 180.0, 50.0 * M_PI / 180.0};
}; 

#endif