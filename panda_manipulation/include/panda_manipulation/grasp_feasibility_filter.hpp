#ifndef PANDA_MANIPULATION__GRASP_FEASIBILITY_FILTER_HPP_
#define PANDA_MANIPULATION__GRASP_FEASIBILITY_FILTER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>

// 一个 GraspNet 输出的候选抓取姿态的最终评估结果
struct GraspEvaluation
{
    // ---- GraspNet 输出位姿的基本信息，包括位姿、分数和夹爪宽度 ----
    geometry_msgs::msg::PoseStamped pose;
    double score = 0.0;
    double width = 0.0;

    // ---- 每一项的评估结果以及最终的可行性结果 ----
    bool is_ik_ok    = false;       // 该位姿的 IK 是否有解
    bool is_joint_ok = false;       // 该位姿的关节行程是否在阈值内(防止机械臂绕大圈)
    bool is_table_ok = false;       // 该位姿是否可能导致夹爪处于桌面以下
    bool is_feasible = false;       // 该位姿是否可行(只有前三项判定全部通过才认为是可行的)

    // ---- 衡量一个抓取位姿优劣的判定指标 ----
    double joint_travel = 0.0;              // 关节总行程，sum |q_ik - q_now|，单位 rad
    double ik_time_ms   = 0.0;              // 本候选解 IK 的耗时
    double rank         = 0.0;              // 该位姿的最终排序分(只有位姿可行才计算)

    // 判定该位姿失败的原因
    std::string reject_reason;
};

// 每一级的淘汰统计，用来输出日志
struct FilterStageStats
{
    int total = 0;               // GraspNet 输出的候选抓取姿态数量
    int preselected = 0;         // 按分数预筛后实际做 IK 的数量
    int table_ok = 0;            // 符合桌面几何条件的位姿数量
    int ik_ok = 0;               // 有 IK 稳定解的数量
    int joint_ok = 0;            // 关节角度变化符合要求的数量
    int feasible = 0;            // 最终具有可行性的解的数量
};

class GraspFeasibilityFilter
{
    public:
        // 构造函数
        explicit GraspFeasibilityFilter(const rclcpp::Node::SharedPtr& node);

        // 对外接口：返回 GraspNet 所有输出的评估结果
        std::vector<GraspEvaluation> graspPoseFilter(
            const std::vector<geometry_msgs::msg::PoseStamped> & candidates,
            const std::vector<double> & scores,
            const std::vector<double> & widths,
            const std::vector<double> & current_joints);
            
    private:
        // 加载本节点的参数
        void loadParameters();

        // 节点初始化：加载机械臂模型、获取 JointModelGroup、检查 IK 求解器是否存在
        bool filterInitialize();

        // 将候选抓取姿态按照分数降序排列，并返回其在原数组中的下标
        std::vector<std::size_t> preselectByScore(const std::vector<double> & scores) const;

        // 对传入的候选抓取姿态使用 IK 求解器，查看是否存在稳定解
        bool solveIK(const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& robot_state, moveit::core::RobotState& pose_solution) const;
        
        // 根据 IK 求解器的解，查看机器人整体或者某一个关节是否需要“绕大圈”才能够实现
        double computeJointTravel(const moveit::core::RobotState& pose_solution, const std::vector<double>& current_joints) const;

        // 估算手指最低点是否会碰到桌面
        bool checkTableClearance(const geometry_msgs::msg::PoseStamped& pose) const;

        // 计算合理的抓取位姿的排名分数
        double computeRank(const GraspEvaluation& graspEval, double max_score) const;

        // 输出调试日志
        void printLog();

        // 工具函数：geometry_msgs::Pose -> Eigen::Isometry3d
        static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose);

        // ----- ROS & MoveIt 句柄 -----
        rclcpp::Node::SharedPtr node_;
        std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
        bool is_node_ready_ = false;

        // ----- 节点中需要使用到的参数 -----
        std::string joint_model_group_name_ = "panda_arm";
        std::string tip_link_ = "panda_link8";
        std::string base_link_ = "panda_link0";

        int max_candidates_ = 12;               // 最多只对前 12 个抓取位姿进行评估，防止计算量过大
        double ik_timeout_ = 0.05;              // 单次 IK 超时（秒）
        double finger_max_width_ = 0.075;       // 夹爪的最大张开宽度
        double table_top_z_ = 0.035;            // panda_link0 下桌面上表面高度
        double table_clearance_ = 0.015;        // 手指最低点离桌面的最小间隙

        double w_score_ = 0.4;
        double w_joint_ = 0.3;
        double w_approach_ = 0.3;

        // 日志统计输出
        FilterStageStats last_stats_;
};

#endif