#include "panda_manipulation/grasp_feasibility_filter.hpp"
#include <algorithm>   
#include <cmath>        
#include <limits>       
#include <numeric>  
#include <string>
#include <vector>
#include <chrono>

// 构造函数
GraspFeasibilityFilter::GraspFeasibilityFilter(const rclcpp::Node::SharedPtr& node) : node_(node)  {
    // 1. 加载本节点中的参数
    loadParameters();

    // 2. 初始化节点
    is_node_ready_ = filterInitialize();
    if (!is_node_ready_) {
        RCLCPP_ERROR(node_->get_logger(), "[FILTER] initialization FAILED, filter will be a no-op");
        return;
    }
}

// ----- 加载本节点的参数 -----
void GraspFeasibilityFilter::loadParameters() {
    // 先定义两个读取 double 和 int 类型参数的 lambda 函数
    auto get_double = [this](const std::string& name, double default_value) -> double {
        if (!node_->has_parameter(name)) {
            return node_->declare_parameter<double>(name, default_value);
        }
        return node_->get_parameter(name).as_double();
    };

    auto get_int = [this](const std::string& name, int default_value) -> int {
        if (!node_->has_parameter(name)) {
            return node_->declare_parameter<int>(name, default_value);
        }
        return node_->get_parameter(name).as_int();
    };

    // 检查节点中是否含有指定的参数，存在则获得其当前值，不存在则设置为默认值
    max_candidates_ = get_int("max_candidates_", 12);
    ik_timeout_ = get_double("ik_timeout_", 0.05);
    finger_max_width_ = get_double("finger_max_width_", 0.075);             
    table_top_z_ = get_double("table_top_z_", 0.035);          
    table_clearance_ = get_double("table_clearance_", 0.015);    
    w_score_ = get_double("w_score_", 0.6);
    w_joint_ = get_double("w_joint_", 0.3);
    w_approach_ = get_double("w_approach_", 0.1);
}

// ----- 节点的初始化函数 -----
bool GraspFeasibilityFilter::filterInitialize() {
    RCLCPP_INFO(node_->get_logger(), "Entering filterInitialize()...");
    // 1. 加载机器人模型，重点是需要获取到 IK 求解器，否则后面的过滤无法计算
    try {
        // 这里的第三个参数一定要设置成 true，否则无法加载 IK 求解器，后续无法计算
        robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
            node_, "robot_description", true);

        // 获取机器人模型
        robot_model_ = robot_model_loader_->getModel();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),"Failed to load robot model: %s", e.what());
        return false;
    }

    if (!robot_model_) {
        RCLCPP_ERROR(node_->get_logger(), "Robot model is null after loading");
        return false;
    }

    // 2. 获取机器人模型对应的 JointModelGroup
    joint_model_group_ = robot_model_->getJointModelGroup(joint_model_group_name_); 
    if (!joint_model_group_) {
        RCLCPP_ERROR(node_->get_logger(), "JointModelGroup '%s' not found in robot model", joint_model_group_name_.c_str());
        return false;
    }

    // 3. 检查 IK 求解器是否真正可用
    // 3.1 首先检查 getSolverInstance() 是否为空
    const auto& ik_solver = joint_model_group_->getSolverInstance();
    if (!ik_solver) {
        RCLCPP_ERROR(node_->get_logger(),
                 "No IK solver instantiated for group '%s'. "
                 "Check kinematics.yaml and that load_kinematics_solvers=true",
                 joint_model_group_name_.c_str());
        return false;
    }

    // 3.2 检查机械臂末端执行器是否在该执行器支持的列表中
    const auto& tip_frames = ik_solver->getTipFrames();
    bool is_tip_link_included = (std::find(tip_frames.begin(), tip_frames.end(), tip_link_) != tip_frames.end());
    if (!is_tip_link_included) {
        RCLCPP_ERROR(node_->get_logger(), "tip_link '%s' not in solver tip frames", tip_link_.c_str());
        return false;
    }

    return true;
}

// ----- 返回 GraspNet 所有输出的评估结果 -----
std::vector<GraspEvaluation> GraspFeasibilityFilter::graspPoseFilter(
            const std::vector<geometry_msgs::msg::PoseStamped> & candidates,
            const std::vector<double> & scores,
            const std::vector<double> & widths,
            const std::vector<double> & current_joints) 
{
    // 返回的评估结果
    std::vector<GraspEvaluation> filterResult; 

    // 1. 前置检查 --- 包括节点是否初始化完成，以及传入的数组是否为空
    if (!is_node_ready_) {
        RCLCPP_ERROR(node_->get_logger(), "[FILTER] not ready, skip filtering (initialization failed earlier)");
        return filterResult;
    }

    if (current_joints.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[FILTER] current_joints is empty, cannot evaluate travel");
        return filterResult;
    }

    // 考虑到传入的候选位姿数量和得分数量可能并不一致，这里以长度较小的数组为准
    const size_t array_len = std::min({candidates.size(), scores.size(), widths.size()});
    if (array_len == 0) {
        // 传入的有空数组
        RCLCPP_ERROR(node_->get_logger(), "[FILTER] candidates or scores is empty");
        return filterResult;
    }

    // 清空上一次的日志输出，获取本次处理的 GraspNet 输出的候选抓取姿态数量
    last_stats_ = FilterStageStats{};
    last_stats_.total = static_cast<size_t>(array_len);

    // 2. 将候选抓取姿态按照分数降序排列，获取分数靠前的位姿
    std::vector<double> score_prefix(scores.begin(), scores.begin() + array_len);
    const std::vector<size_t> index_array = preselectByScore(score_prefix);
    if (index_array.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[FILTER] No valid candidates selected after score preselection");
        return filterResult;
    }
    last_stats_.preselected = static_cast<size_t>(index_array.size());

    // 3. 机械臂当前的真实关节角度填进一个临时的状态对象里，作为 IK 求解的基准种子
    moveit::core::RobotState robot_state(robot_model_);                      // 创建代表机械臂完整关节状态的容器对象
    robot_state.setToDefaultValues();                                        // 所有关节初始化为默认值
    robot_state.setJointGroupPositions(joint_model_group_, current_joints);  // 替换为当前实际读到的关节角
    robot_state.update();                                                    // 同步机械臂当前姿态

    // 4.1 计算 GraspNet 输出位姿的最大值，用于归一化其输出
    auto it = std::max_element(scores.begin(), scores.begin() + array_len);
    double max_score = (it != scores.begin() + array_len && *it > 0.0) ? *it : 1.0;

    // 4.2 计算 model_frame 与 panda_link0 之间的变换关系
    // 计算的原因是 setFromIK 默认是相对 URDF 根坐标系，项目中即为 base_footprint；而传入的位姿是相对 panda_link0 的
    // 我们现在需要求解 panda_link8 在 base_footprint 下的 solveIK 是否有解
    // base_footprint_to_link8 = base_footprint_to_link0 * link0_to_grasptcp * grasptcp_to_tcp * tcp_to_link8
    // 首先获取 base_footprint_to_link0
    const Eigen::Isometry3d tf_model_to_link0 = robot_state.getGlobalLinkTransform(base_link_);

    // 其次获取 link0_to_graspnetTCP，这个 TF 是固定的
    Eigen::Isometry3d tf_grasptcp_to_tcp = Eigen::Isometry3d::Identity();
    tf_grasptcp_to_tcp.linear() <<
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0;

    // 最后获取 tcp_to_link8，利用 robot model 求相对变换
    const Eigen::Isometry3d tf_model_to_link8 = robot_state.getGlobalLinkTransform(tip_link_);
    const Eigen::Isometry3d tf_model_to_tcp = robot_state.getGlobalLinkTransform("grasp_tcp");
    const Eigen::Isometry3d tf_tcp_to_link8 = tf_model_to_tcp.inverse() * tf_model_to_link8;

    // 5. 对每个抓取位姿进行评估
    for (std::size_t index : index_array) {
        // 5.1 构造评估的结果
        GraspEvaluation graspEval;
        graspEval.pose = candidates[index];
        graspEval.score = scores[index];
        graspEval.width = widths[index];

        // 5.2 检查该位姿是否处于桌面以下
        if (!checkTableClearance(graspEval.pose)) {
            graspEval.reject_reason = "below_table";
            continue;
        }
        graspEval.is_table_ok = true;
        last_stats_.table_ok++;

        // 5.3 使用 IK 求解器查看是否存在稳定解
        const Eigen::Isometry3d target_pose = tf_model_to_link0 * poseToEigen(graspEval.pose.pose) * tf_grasptcp_to_tcp * tf_tcp_to_link8;
        moveit::core::RobotState pose_solution(robot_model_);
        const auto ik_start_time = std::chrono::steady_clock::now();
        if (!solveIK(target_pose, robot_state, pose_solution)) {
            graspEval.reject_reason = "no_ik";
            continue;
        }
        graspEval.ik_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - ik_start_time).count();
        graspEval.is_ik_ok = true;
        last_stats_.ik_ok++;

        // 5.4 对机器人的关节行程进行检查
        graspEval.joint_travel = computeJointTravel(pose_solution, current_joints);
        graspEval.is_joint_ok = true;
        last_stats_.joint_ok++;

        // 5.5 通过上述检测的认为改位姿是合适的，计算其最终的排名分并追加进结果数组
        graspEval.is_feasible = true;
        graspEval.rank = computeRank(graspEval, max_score);
        filterResult.push_back(graspEval);
    }

    // 6. 根据分数排名对可行解进行降序排名
    std::sort(filterResult.begin(), filterResult.end(),
            [](const GraspEvaluation& a, const GraspEvaluation& b){
                return a.rank > b.rank;
            });
    last_stats_.feasible = static_cast<size_t>(filterResult.size());
    
    // 7. 输出日志方便调试
    printLog();

    return filterResult;
}   

// ----- 将候选抓取姿态按照分数降序排列，并返回其在原数组中的下标 -----
std::vector<std::size_t> GraspFeasibilityFilter::preselectByScore(const std::vector<double> & scores) const{
    // 1. 边界情况：如果传入的数组为空，不做任何处理
    const size_t len = scores.size();
    if (len == 0) {
        return {};
    }

    // 将候选抓取姿态按照分数降序排列
    // 2.1 生成 0 ~ n-1 的下标
    std::vector<std::size_t> indices(len);
    std::iota(indices.begin(), indices.end(), 0);

    // 2.2 根据声明的参数，只对分数处于前 Top-K 的候选位姿进行评估
    const size_t candidates_len = std::min(len, static_cast<size_t>(max_candidates_));

    // 2.3 按照分数降序排列
    std::partial_sort(indices.begin(), indices.begin() + candidates_len, indices.end(),
                      [&scores](std::size_t a, std::size_t b) {
                        return scores[a] > scores[b];
                      });

    // 2.4 只保留分数处于前 Top-K 的候选位姿
    indices.resize(candidates_len);

    return indices;
}

// ----- 对传入的候选抓取姿态使用 IK 求解器，查看是否存在稳定解 -----
bool GraspFeasibilityFilter::solveIK(const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& robot_state, moveit::core::RobotState& pose_solution) const {
    // 1. 后续的 setFromIK 操作会原地修改传入的机器人状态，故此处必须要先进行拷贝操作
    pose_solution = robot_state;

    // 2. 对传入的候选抓取姿态使用 IK 求解器
    const bool has_found_solution = pose_solution.setFromIK(
        joint_model_group_,              
        target_pose,  
        tip_link_,                                    
        ik_timeout_,                    
        moveit::core::GroupStateValidityCallbackFn()  // 无额外有效性回调
    );

    if (!has_found_solution) {
        return false;
    }

    // 3. 检查解是否满足关节限位
    if (!pose_solution.satisfiesBounds(joint_model_group_)) {
        return false;
    }

    // 4. 更新内部变换缓存，方便后续计算
    pose_solution.update();

    return true;
}

// ----- 根据 IK 求解器的解，查看机器人整体或者某一个关节是否需要“绕大圈”才能够实现 -----
double GraspFeasibilityFilter::computeJointTravel(const moveit::core::RobotState & pose_solution, const std::vector<double> & current_joints) const {
    // 1. 先从 IK 解中获取每个关节的角度
    std::vector<double> q_solution;
    pose_solution.copyJointGroupPositions(joint_model_group_, q_solution);

    // 2. 如果 IK 解获得的维度与机器人当前关节维度不一致，认为错误
    if (q_solution.size() != current_joints.size()) {
        return std::numeric_limits<double>::infinity();
    }

    // 3. 计算从当前位置移动至最终位置，单个关节和所有关节的移动角度
    double all_joint_angle = 0.0;
    for (size_t i = 0; i < q_solution.size(); i++) {
        const double curr_angle = std::abs(q_solution[i] - current_joints[i]);
       
        // 累加所有关节的移动角度
        all_joint_angle += curr_angle;
    }
    
    return all_joint_angle;
}

// ----- 估算手指最低点是否会碰到桌面 -----
bool GraspFeasibilityFilter::checkTableClearance(const geometry_msgs::msg::PoseStamped& pose) const {
    // 1. 先获取夹爪张开方向(Y轴)
    const Eigen::Vector3d dir_open = poseToEigen(pose.pose).linear().col(1);

    // 2. 先计算 TCP 的位置
    const double z_tcp = pose.pose.position.z;

    // 3. 考虑最极端的情况，当夹爪完全打开时，计算相对较低的那一根手指的高度
    const double z_finger_low = z_tcp - 0.5 * finger_max_width_ * std::abs(dir_open.z());

    // 4. 与桌面安全高度比较
    const double z_min_allowed = table_top_z_ + table_clearance_;
    if (z_finger_low < z_min_allowed) {
        return false;
    }
    return true;
}

// ----- 计算可行抓取位姿的排序分 -----
double GraspFeasibilityFilter::computeRank(const GraspEvaluation& graspEval, double max_score) const {
    // 1. 先将本批内的分数进行归一化
    const double r_score = max_score > 0.0 ? (graspEval.score / max_score) : 0.0;

    // 2. 归一化关节角度，越接近原始状态越好
    const double r_joint = 1.0 / (1.0 + std::max(0.0, graspEval.joint_travel));

    // 3. 角度偏好：考虑到物体摆放在桌面上，优先考虑从上往下的夹取位姿
    const double r_approach = -poseToEigen(graspEval.pose.pose).linear().col(0).z();

    // 返回最终的加权得分
    return w_score_ * r_score + w_joint_ * r_joint + w_approach_ * r_approach;
}

// ----- 输出调试日志 -----
void GraspFeasibilityFilter::printLog() {
    RCLCPP_INFO(node_->get_logger(),
        "[FILTER] %d candidates -> preselected %d -> table_ok %d -> ik_ok %d -> joint_ok %d -> feasible %d",
        last_stats_.total,
        last_stats_.preselected,
        last_stats_.table_ok,
        last_stats_.ik_ok,
        last_stats_.joint_ok,
        last_stats_.feasible);
}

// ----- 工具函数：geometry_msgs::Pose -> Eigen::Isometry3d -----
Eigen::Isometry3d GraspFeasibilityFilter::poseToEigen(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // 获取位姿的四元数并归一化
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    q.normalize();

    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

    return T;
}