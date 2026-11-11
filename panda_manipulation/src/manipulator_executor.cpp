#include <algorithm>
#include <cmath>
#include <map>
#include <deque> 
#include "panda_manipulation/manipulator_executor.hpp"

// 构造函数实现
ManipulatorExecutor::ManipulatorExecutor(const rclcpp::Node::SharedPtr& node) : node_(node) {
    // 1.1 保存两个控制组的句柄
    using namespace moveit::planning_interface;
    arm_group_ = std::make_shared<MoveGroupInterface>(node_, "panda_arm");   
    hand_group_ = std::make_shared<MoveGroupInterface>(node_, "hand");

    // 1.2 设置 panda_arm 规划组运动时相对于基坐标系运动
    arm_group_->setPoseReferenceFrame("panda_link0");

    // 2. 设置最大速度与加速度比例
    arm_group_->setMaxVelocityScalingFactor(0.5);
    arm_group_->setMaxAccelerationScalingFactor(0.5);

    // 3. 初始化 TF2 监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. 初始化 Moveit 场景规划器
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // 5. 初始化抓取物体位置订阅器
    object_pose_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/world/empty_world/pose/info",
        10,
        std::bind(&ManipulatorExecutor::objectPoseCallback, this, std::placeholders::_1)
    );

    // 6. 初始化夹爪位置订阅器
    finger_joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10),
        std::bind(&ManipulatorExecutor::fingerPoseCallback, this, std::placeholders::_1)
    );
}

// ----- 控制机械臂从物料台上按照指定位姿抓取物体 -----
bool ManipulatorExecutor::pickExecutor(const geometry_msgs::msg::PoseStamped& pick_pose) {
    // 1. 先判断当前时刻机器人是否可用
    // if (!isRobotReady()) return false;

    // 2. 判断感知模块传入的抓取位姿是否合理，并转换到获得 Link0 坐标系下末端执行器 Link8 的位姿
    geometry_msgs::msg::PoseStamped pick_pose_link0;
    if (!validateAndTransformPose(pick_pose, pick_pose_link0, "pick")) return false;

    // 3. 对转换后的抓取位姿进行有效性判断
    if (!isPoseValidate(pick_pose_link0)) return false;

    // 4. 三段式抓取物体
    // 4.1 根据抓取位姿自身的姿态提取下探方向(TCP 下探方向为 Z 轴)
    tf2::Quaternion q;
    tf2::fromMsg(pick_pose_link0.pose.orientation, q);
    tf2::Matrix3x3 rot_matrix(q);
    tf2::Vector3 approach_dir = rot_matrix.getColumn(2);

    // 4.2 定义一个偏移函数，其中 d > 0 表示沿着接近物体的方向走，d < 0 表示沿着远离物体的方向走
    auto offset_along = [&](const geometry_msgs::msg::PoseStamped& pose_base, double d) {
        geometry_msgs::msg::PoseStamped p = pose_base;
        p.pose.position.x += d * approach_dir.x();
        p.pose.position.y += d * approach_dir.y();
        p.pose.position.z += d * approach_dir.z();
        return p;
    };

    // 4.3 定义三段式抓取中的高度参数
    const double hover_height   = 0.30;    // 高空悬停：距抓取点 30cm 的空域
    const double pregrasp_above = 0.12;    // 预抓取：指尖悬在抓取点上方 12cm

    // 4.4 三段式第一段：先运动至物体上方安全距离处
    if (!moveArmToPose(offset_along(pick_pose_link0, -hover_height), false)) return false;

    // 4.5 在悬停位置打开夹爪
    if (!controlGripper(true)) return false;

    // 4.6 三段式第二段：直线下降到预抓取点
    if (!moveArmToPose(offset_along(pick_pose_link0, -pregrasp_above), true)) return false;

    // 4.7 三段式第三段：继续直线下降到真正的抓取点
    arm_group_->setMaxVelocityScalingFactor(0.1);
    arm_group_->setMaxAccelerationScalingFactor(0.1);
    if (!moveArmToPose(pick_pose_link0, true)) return false;

    // 5.1 获取此时物体的 Z 轴位置
    // 获取物体位置的原因是，仿真中比较难做到像实机中那样用 effort 来判定是否抓取住了物体
    // 参考如 robosuite 等开源项目中的常见做法，采用比较抓取前后物体在 Z 方向的位置来判定物体成功抓取
    geometry_msgs::msg::Point initial_pose;
    if (!getObjectLatestPose(initial_pose)) return false;

    // 5.2 步进式关闭夹爪抓取物体
    // 采用该方法的原因为 GraspNet 输出的宽度并不是物体的实际宽度，而完全闭合虽然也可以，但会导致夹爪的位置大幅度波动
    // 为了尽可能贴近实机的抓取思路，采用步进式抓取，当夹爪的位置不再随着指令减小时，认为抓取到了物体
    if (!closeGripperStepwise()) return false;

    // 6.1 先控制机械臂抬升一定距离，然后再沿抓取方向退回一定距离，防止物体和桌面挂蹭或碰撞
    geometry_msgs::msg::PoseStamped lift_target_pose = pick_pose_link0;
    lift_target_pose.pose.position.z += 0.05;
    if (!moveArmToPose(lift_target_pose, true)) return false;
    if (!moveArmToPose(offset_along(pick_pose_link0, -0.25), true)) return false;

    // 6.2 回到 Home 位置，获取此时物体的 Z 轴位置进行判定
    arm_group_->setMaxVelocityScalingFactor(0.5);
    arm_group_->setMaxAccelerationScalingFactor(0.5);
    if (!moveArmToJoints(home_joints)) return false;

    geometry_msgs::msg::Point lift_pose;
    if (!getObjectLatestPose(lift_pose)) return false;
    return lift_pose.z - initial_pose.z > 0.04;
}

// ----- 控制机械臂向放置台上放置物体 -----
bool ManipulatorExecutor::placeExecutor(const geometry_msgs::msg::PoseStamped& place_pose) {
    // 1. 先判断当前时刻机器人是否可用
    // if (!isRobotReady()) return false;

    // 2. 将感知模块传入的抓取位姿转换到 panda_link0 坐标系下
    geometry_msgs::msg::PoseStamped place_pose_link0;
    if (!validateAndTransformPose(place_pose, place_pose_link0, "place")) return false;

    // 3. 对转换后的抓取位姿进行有效性判断
    if (!isPoseValidate(place_pose_link0)) return false;

    // 4. 第一段：运动到托盘上方的悬停点
    geometry_msgs::msg::PoseStamped hover_pose = place_pose_link0;
    hover_pose.pose.position.z += 0.15;
    if (!moveArmToPose(hover_pose)) return false;

    // 7. 第二段：沿着竖直方向直线下降到放置点
    if (!descendArmStepwise(hover_pose)) return false;

    // 8. 张开夹爪松开物体，等待物体稳定落下
    if (!controlGripper(true)) return false;
 
    // 9. 第三段：竖直抬起 10cm，脱离托盘
    geometry_msgs::msg::PoseStamped retreat_pose = place_pose_link0;
    retreat_pose.pose.position.z += 0.10;
    arm_group_->setMaxVelocityScalingFactor(0.5);
    arm_group_->setMaxAccelerationScalingFactor(0.5);
    if (!moveArmToPose(retreat_pose, true)) return false;

    // 10. 回到 Home 姿态，并获取当前时刻物体的位姿
    if (!moveArmToJoints(home_joints)) return false;

    return true;
}

// ----- 控制机械臂运动至指定位置 -----
bool ManipulatorExecutor::armMoveExecutor(const geometry_msgs::msg::PoseStamped& arm_pose) {
    // 1. 先判断当前时刻机器人是否可用
    // if (!isRobotReady()) return false;

    // 2. 将感知模块传入的抓取位姿转换到 panda_link0 坐标系下(为了复用之前的代码，考虑到这里没有涉及 GraspNet，传入 place 参数)
    geometry_msgs::msg::PoseStamped arm_pose_link0;
    if (!validateAndTransformPose(arm_pose, arm_pose_link0, "place")) return false;

    // 3. 对转换后的抓取位姿进行有效性判断
    if (!isPoseValidate(arm_pose_link0)) return false;

    // 4. 控制机械臂运动至指定位置
    if (!moveArmToPose(arm_pose_link0)) return false;

    return true;
}

// 在 Pick 流程中向 Moveit 的规划场景中添加物体，防止机械臂与其发生碰撞
bool ManipulatorExecutor::preparePickCollisionScene(double center_x, double center_y) {
    // 1. 向 Moveit 的规划场景中添加桌子
    const std::string pick_table_id = "pick_table";

    // 2. 设置桌子的三维大小
    double table_x = pick_table_size_x_;
    double table_y = pick_table_size_y_;
    double table_z = table_thickness_;

    // 3. 设置桌子在 panda_link0 下的具体位置
    geometry_msgs::msg::Pose pick_table_pose;
    pick_table_pose.position.x = center_x;
    pick_table_pose.position.y = center_y;
    pick_table_pose.position.z = table_top_z_ - table_thickness_ / 2.0;
    pick_table_pose.orientation.w = 1.0;
    
    // 4. 向 Moveit 的规划场景中添加桌子
    return addBoxToScene(pick_table_id, pick_table_pose, table_x, table_y, table_z);
}

// 在 Place 流程中向 Moveit 的规划场景中添加物体，防止机械臂与其发生碰撞
bool ManipulatorExecutor::preparePlaceCollisionScene(double center_x, double center_y, double center_z) {
    // 这里主要向 Moveit 中添加需要放置托盘和桌面
    // 1. 向 Moveit 的规划场景中添加托盘和桌子
    const std::string plate_id = "plate";
    const std::string place_table = "place_table";

    // 2.1 设置桌子的三维大小
    double table_x = place_table_size_x_;
    double table_y = place_table_size_y_;
    double table_z = table_thickness_;

    // 2.2 设置托盘的三维大小
    double plate_x = plate_size_;
    double plate_y = plate_size_;
    double plate_z = plate_thickness_;

    // 3.1 设置桌子在 panda_link0 下的具体位置
    geometry_msgs::msg::Pose place_table_pose;
    place_table_pose.position.x = center_x;
    place_table_pose.position.y = center_y;
    place_table_pose.position.z = table_top_z_ - table_thickness_ / 2.0;
    place_table_pose.orientation.w = 1.0;

    // 3.2 设置托盘在 panda_link0 下的具体位置
    geometry_msgs::msg::Pose plate_pose;
    plate_pose.position.x = center_x;
    plate_pose.position.y = center_y;
    plate_pose.position.z = center_z - plate_thickness_ / 2;
    plate_pose.orientation.w = 1.0;

    // 4. 向 Moveit 的规划场景中进行添加
    bool is_place_table_success = addBoxToScene(place_table, place_table_pose, table_x, table_y, table_z);
    bool is_plate_success = addBoxToScene(plate_id, plate_pose, plate_x, plate_y, plate_z);
    return is_place_table_success && is_plate_success;
}

// 向 Moveit 的规划场景中删除 Pick 流程中所添加的物体
bool ManipulatorExecutor::removePickCollisionScene() {
    // 删除 Pick 流程中所添加的物体
    const std::string pick_table_id = "pick_table";
    return removeObjectFromScene(pick_table_id);
}

// 向 Moveit 的规划场景中删除 Place 流程中所添加的物体
bool ManipulatorExecutor::removePlaceCollisionScene() {
    // 删除 Place 流程中所添加的物体
    const std::string place_table_id = "place_table";
    const std::string plate_id = "plate";

    bool is_table_success = removeObjectFromScene(place_table_id);
    bool is_plate_success = removeObjectFromScene(plate_id);
    return is_table_success && is_plate_success;
}

// 向 Moveit 的规划场景中添加长方体物体
bool ManipulatorExecutor::addBoxToScene(const std::string & box_id, geometry_msgs::msg::Pose & box_pose, double box_x, double box_y, double box_z) {
    // 1. 向 Moveit 的规划场景中添加长方体物体
    moveit_msgs::msg::CollisionObject box_to_add;
    box_to_add.header.frame_id = "panda_link0";
    box_to_add.id = box_id;

    // 2. 设置该长方体的三维长度
    shape_msgs::msg::SolidPrimitive box_to_add_size;
    box_to_add_size.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_to_add_size.dimensions.resize(3);
    box_to_add_size.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box_x;
    box_to_add_size.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box_y;
    box_to_add_size.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box_z;

    // 3. 设置该长方体在 panda_link0 下的具体位置
    geometry_msgs::msg::Pose box_to_add_pose;
    box_to_add_pose.position.x = box_pose.position.x;
    box_to_add_pose.position.y = box_pose.position.y;
    box_to_add_pose.position.z = box_pose.position.z;
    box_to_add_pose.orientation.w = box_pose.orientation.w;

    // 4. 将长方体放入 Moveit 的规划场景之中
    box_to_add.primitives.push_back(box_to_add_size);
    box_to_add.primitive_poses.push_back(box_to_add_pose);
    box_to_add.operation = moveit_msgs::msg::CollisionObject::ADD;
    return planning_scene_interface_->applyCollisionObject(box_to_add);
}

// 向 Moveit 的规划场景中删除物体
bool ManipulatorExecutor::removeObjectFromScene(const std::string & object_id) {
    moveit_msgs::msg::CollisionObject objetc_to_remove;
    objetc_to_remove.header.frame_id = "panda_link0";
    objetc_to_remove.id = object_id;

    objetc_to_remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    return planning_scene_interface_->applyCollisionObject(objetc_to_remove);
}

// 获取当前机械臂的所有关节角度状态
std::vector<double> ManipulatorExecutor::getCurrentJointValues() {
    return arm_group_->getCurrentJointValues();
}

// ----------------------------------------
// ----------- 机械臂运动辅助方法 ------------
// ----------------------------------------
// 控制机械臂移动到指定位姿
bool ManipulatorExecutor::moveArmToPose(const geometry_msgs::msg::PoseStamped & target_pose, bool use_cartesian) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    if (use_cartesian) {
        // --- 笛卡尔直线路径规划 ---
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        double eef_step = 0.0025;       
        double jump_threshold = 0.0;    

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(node_->get_logger(), "Cartesian path coverage: %.2f%%", fraction * 100.0);

        if (fraction >= 0.99) {
            plan.trajectory_ = trajectory;
            success = true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian planning failed (coverage only %.2f%%)", fraction * 100.0);
            return false;
        }
    } else {
        // --- 普通自由路径规划 ---
        arm_group_->setStartStateToCurrentState();
        arm_group_->setPlanningTime(5.0);              
        arm_group_->setNumPlanningAttempts(10);
        arm_group_->clearPoseTargets();
        arm_group_->setPoseTarget(target_pose.pose);
        success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // 执行轨迹
    if (success) {
        auto exec_result = arm_group_->execute(plan);
        return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Move Arm To Pose Planning Failed!");
        return false;
    }
}

// 控制机械臂运动到指定关节角度位置
bool ManipulatorExecutor::moveArmToJoints(const std::vector<double>& joint_positions) {
    arm_group_ -> setJointValueTarget(joint_positions);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        auto exec_result = arm_group_->execute(plan);
        return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Move Arm To Joints Planning Failed!");
        return false;
    }
}

// 控制夹爪的开与闭
bool ManipulatorExecutor::controlGripper(bool open) {
    // 确定让夹爪开启还是关闭
    std::string target_state = open ? "open" : "close";

    if (!hand_group_ -> setNamedTarget(target_state)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set named target: %s", target_state.c_str());
        return false;
    }
    auto result = hand_group_->move();
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

// 控制夹爪步进运动
bool ManipulatorExecutor::closeGripperStepwise() {
    // 单步步进长度 --- 2mm
    const double stepwise = 0.002;

    // 1. 先获取夹爪的当前位置；位置过小，不够单次步进的话直接跳过
    double finger_pose = 0.0;
    if (!getFingerLatestPose(finger_pose) || finger_pose < stepwise) return false;

    // 2. 夹爪步进闭合，直到指令变小但夹爪关节不跟(夹住了物体)，或者夹爪基本完全闭合(没有夹住物体)
    bool is_contacted = false;
    hand_group_->setMaxVelocityScalingFactor(1.0);
    hand_group_->setMaxAccelerationScalingFactor(1.0);

    for (; finger_pose > stepwise; finger_pose -= stepwise) {
        // 控制夹爪步进运动
        hand_group_->setJointValueTarget(std::map<std::string, double>{{"panda_finger_joint1", finger_pose}});
        if (hand_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) return false;

        // 获取运动后的夹爪位置进行比较，如果指令变小但是夹爪的实际位置没有变小，说明夹住了物体
        double curr_finger_pose = 0.0;
        if (!getFingerLatestPose(curr_finger_pose)) return false;

        if (std::abs(curr_finger_pose - finger_pose) > 0.008) {
            is_contacted = true;
            break;
        }
    }

    return is_contacted;
}

// 控制机械臂步进下降
bool ManipulatorExecutor::descendArmStepwise(geometry_msgs::msg::PoseStamped & initial_pose) {
    const double stepwise = 0.005;         // 单步下降的距离
    const double contact_eps = 0.0025;     // 当物体的实际下降距离小于此数时，认为已经接触到了桌面
    const double slip_eps = 0.015;         // 当物体的实际下降距离大于此数时，认为物体从夹爪中脱落
    const int max_steps = 40;              // 最大步进，超过最大次数还没有结束则认为下方没有桌面
    const int deque_len = 3;               // 队列长度，用于求取平均值以消除下降过程中的滞后和抖动

    // 1. 先获取物体的当前位置，定义存储每次下降距离的队列
    geometry_msgs::msg::Point prev_object_pose;
    if (!getObjectLatestPose(prev_object_pose)) return false;
    std::deque<double> descend_window;

    // 2. 机械臂步进下降，直到指令变小但夹机械臂不跟(物体已经到达桌面上方)
    arm_group_->setMaxVelocityScalingFactor(0.2);
    arm_group_->setMaxAccelerationScalingFactor(0.2);

    for (int i = 0; i < max_steps; i++) {
        // 控制机械臂步进运动
        initial_pose.pose.position.z -= stepwise;

        // 如果笛卡尔直线运动失败，说明物体距离桌面很近
        if (!moveArmToPose(initial_pose, true)) {
            RCLCPP_WARN(node_->get_logger(), "Cartesian descent blocked, treat as contact");
            return true;
        }

        // 获取当前时刻物体的位置
        geometry_msgs::msg::Point curr_object_pose;
        if (!getObjectLatestPose(curr_object_pose)) return false;

        // 计算此次步进运动的下降距离
        const double descend_distance = prev_object_pose.z - curr_object_pose.z;
        descend_window.push_back(descend_distance);
        if (descend_window.size() > static_cast<size_t>(deque_len)) descend_window.pop_front();

        // 计算平均值
        double descend_mean = 0.0;
        for (auto & item : descend_window) descend_mean += item;
        descend_mean /= static_cast<double>(descend_window.size()); 

        // 如果物体下降的距离小于阈值，认为已经接触到了桌面
        if (descend_mean <= contact_eps) return true;  

        // 如果物体的下降距离大于阈值，认为物体从夹爪中脱落
        if (descend_mean >= slip_eps) {
            RCLCPP_ERROR(node_->get_logger(), "Object slipped during descent");
            return false;
        }

        prev_object_pose = curr_object_pose;
    }

    return false;
}

// 安全获取两个坐标系之间的 TF 变换
bool ManipulatorExecutor::getTransform(const std::string& target_frame, const std::string& source_frame, tf2::Transform& transform_out) {
    // 尝试获取两个坐标系之间的变换
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame,
            source_frame,
            tf2::TimePointZero,
            std::chrono::seconds(5)
        );

        // 获取成功后转换为 tf2::Transform，方便后续处理
        tf2::fromMsg(transform_stamped.transform, transform_out);
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Can not get transform from [%s] to [%s]: %s", 
                        source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

// ---------------------------------------------
// ----------- 机械臂任务执行合法性判断 ------------
// ---------------------------------------------
// 检查机械臂控制链路是否具备执行条件
bool ManipulatorExecutor::isRobotReady() {
    // 1. 检查机械臂和夹爪的控制器是否初始化
    if (!arm_group_ || ! hand_group_) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: Arm or Gripper is not initialized");
        return false;
    }

    // 2. 检查能否获取机械臂当前状态
    auto current_state = arm_group_->getCurrentState(2.0);
    if (!current_state) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: failed to get current robot state");
        return false;
    }

    // 3. 检查机器人全局模型是否存在
    const auto& robot_model = current_state->getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: robot model is unavailable");
        return false;
    }

    // 4. 检查规划组是否存在
    const auto* joint_model_group = robot_model->getJointModelGroup(arm_group_->getName());
    if (!joint_model_group) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: planning group not found");
        return false;
    }

    // 5. 检查机械臂当前关节值是否有效
    const auto joint_values = arm_group_->getCurrentJointValues();
    const auto joint_names = arm_group_->getJointNames();

    // 如果没有机械臂的关节数据，或者读到的关节数据与关节数量不一致，认为是错误的
    if (joint_values.empty() || joint_values.size() != joint_names.size()) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: invalid joint state");
        return false;
    }

    // 如果读到的某个关节数据是非法的(NaN 或者 Inf)，认为是错误的
    for (std::size_t i = 0; i < joint_values.size(); ++i) {
        if (!std::isfinite(joint_values[i])) {
            RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: joint %s has invalid value", joint_names[i].c_str());
            return false;
        }
    }

    // 6. 检查机械臂当前各个关节的实际角度是否超出了物理机械极限
    if (!current_state->satisfiesBounds(joint_model_group)) {
        RCLCPP_ERROR(node_->get_logger(), "Robot is not ready: current state violates joint bounds");
        return false;
    }

    return true;
}

// 判断是否能够进行 TF 转换，并将感知模块传入的位姿抓取位置转换到 panda_link0 坐标系下
bool ManipulatorExecutor::validateAndTransformPose(const geometry_msgs::msg::PoseStamped& input_pose_stamped, geometry_msgs::msg::PoseStamped& output_pose_stamped, const std::string task_stage) {
    // 1. 首先对传入数据的坐标系进行有效性检查
    const std::string& source_frame = input_pose_stamped.header.frame_id;
    const std::string& pick_camera_frame = "pick_camera_depth_optical_frame";
    const std::string& place_camera_frame = "place_camera_depth_optical_frame";
    const std::string& hand_camera_frame = "hand_camera_depth_optical_frame";
    const std::string& link_frame = "panda_link0";
    if (source_frame.empty() || (source_frame != pick_camera_frame && source_frame != place_camera_frame && source_frame != hand_camera_frame && source_frame != link_frame)) {
        RCLCPP_ERROR(node_->get_logger(), "Pose transform failed: input frame_id is empty or error!");
        return false;
    }

    // 2. 对传入的数据进行初步验证，防止在坐标变换出现问题
    const auto & pos = input_pose_stamped.pose.position;
    const auto & ori = input_pose_stamped.pose.orientation;
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z) ||
        !std::isfinite(ori.x) || !std::isfinite(ori.y) || !std::isfinite(ori.z) || !std::isfinite(ori.w)) {
        RCLCPP_ERROR(node_->get_logger(), "Pose transform failed: input pose contains NaN or Inf values");
        return false;
    }

    // 3. 对传入的任务阶段进行验证,其必须为 pick 或者 place
    if (task_stage != "pick" && task_stage != "place") {
        RCLCPP_ERROR(node_->get_logger(), "Task stage error: input task can not be [%s], must be pick or place!", task_stage.c_str());
        return false;
    }

    // 4. 检查是否能够查询 link0 坐标系下 camera 坐标系，和 Grasp_TCP 坐标系下 Link8 的相对位姿
    if (!tf_buffer_->canTransform("panda_link0", source_frame, tf2::TimePointZero, std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(
            node_->get_logger(), 
            "Pose transform failed: TF tree cannot transform from [%s] to [%s]", 
            source_frame.c_str(), "panda_link0"
        );
        return false;
    }

    if (!tf_buffer_->canTransform("grasp_tcp", "panda_link8", tf2::TimePointZero, std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(
            node_->get_logger(), 
            "Pose transform failed: TF tree cannot transform from [%s] to [%s]", 
            "panda_link8", "grasp_tcp"
        );
        return false;
    }

    // 5. 获取 link0 坐标系下 camera 坐标系，和 Grasp_TCP 坐标系下 Link8 的相对位姿
    tf2::Transform tf_link0_to_camera;
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            "panda_link0",                            
            source_frame,         
            tf2::TimePointZero, 
            std::chrono::seconds(1)
        );

        // 转换为 tf2::Transform 供后续矩阵运算使用
        tf2::fromMsg(transform_stamped.transform, tf_link0_to_camera);
    } catch (const tf2::TransformException & ex){
        RCLCPP_ERROR(node_->get_logger(), "Can not get transform: %s", ex.what());
        return false;
    }

    tf2::Transform tf_tcp_to_link8;
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            "grasp_tcp",          
            "panda_link8",        
            tf2::TimePointZero, 
            std::chrono::seconds(1)
        );

        // 转换为 tf2::Transform 供后续矩阵运算使用
        tf2::fromMsg(transform_stamped.transform, tf_tcp_to_link8);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Can not get transform: %s", ex.what());
        return false;
    }

    // 6. 根据输入的任务阶段不同，分别计算相应的相机到 TCP 的变换关系
    tf2::Transform tf_link0_to_link8;
    if (task_stage == "pick") {
        // pick 工作阶段需要构建 Graspnet TCP 坐标系与自定义 TCP 坐标系之间的变换关系
        // 因为 Graspnet TCP 坐标系与自定义的 TCP 坐标系不同，前者以 X 轴作为夹爪下探方向，而后者则以 Z 轴为下探方向
        tf2::Transform tf_link0_to_grasptcp;
        tf2::fromMsg(input_pose_stamped.pose, tf_link0_to_grasptcp);

        static const tf2::Matrix3x3 rot_matrix(
            0.0, 0.0, 1.0,  
            0.0, 1.0, 0.0,  
            -1.0, 0.0, 0.0
        );
        tf2::Transform tf_grasptcp_to_tcp;
        tf_grasptcp_to_tcp.setBasis(rot_matrix);
        tf_grasptcp_to_tcp.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

        // 组合得到最终的相机到自定义 TCP 的变换
        tf_link0_to_link8 = tf_link0_to_grasptcp * tf_grasptcp_to_tcp * tf_tcp_to_link8;
    } else if (task_stage == "place") {
        // place 工作阶段不需要构建 Graspnet TCP 坐标系与自定义 TCP 坐标系之间的变换关系，按照相机指定的位姿即可
        tf2::Transform tf_camera_to_tcp;
        tf2::fromMsg(input_pose_stamped.pose, tf_camera_to_tcp);
        tf_link0_to_link8 = tf_link0_to_camera * tf_camera_to_tcp * tf_tcp_to_link8;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown task stage: %s", task_stage.c_str());
        return false;
    }

    // 8. 统一将结果转回 geometry_msgs 供 MoveIt 使用
    output_pose_stamped.pose.position.x = tf_link0_to_link8.getOrigin().x();
    output_pose_stamped.pose.position.y = tf_link0_to_link8.getOrigin().y();
    output_pose_stamped.pose.position.z = tf_link0_to_link8.getOrigin().z();
    output_pose_stamped.pose.orientation = tf2::toMsg(tf_link0_to_link8.getRotation());

    output_pose_stamped.header.frame_id = "panda_link0";
    output_pose_stamped.header.stamp = input_pose_stamped.header.stamp;

    return true;
}

// 判断感知模块传入的位姿抓取位置是否合法
bool ManipulatorExecutor::isPoseValidate(const geometry_msgs::msg::PoseStamped& target_pose) {
    // 1. 判断位姿抓取位置的 XYZ 是否为有限数值，其不能是 NaN 或者 Inf
    const auto& pos = target_pose.pose.position;
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) return false;

    // 2. 检查四元数是否为有限数值，且模长是否接近 1
    const auto& ori = target_pose.pose.orientation;
    if (!std::isfinite(ori.x) || !std::isfinite(ori.y) || 
        !std::isfinite(ori.z) || !std::isfinite(ori.w)) {
        return false;
    }

    // 考虑到传感器及视觉算法所产生的误差，设定容差，当四元数模长不超过容差时认为是有效的
    double norm_squared = (ori.x * ori.x) + (ori.y * ori.y) + (ori.z * ori.z) + (ori.w * ori.w);
    const double norm_tolerance = 1e-2;
    if (abs(norm_squared - 1.0) > norm_tolerance) return false;

    return true;
}

bool ManipulatorExecutor::moveHome() {
    if (!moveArmToJoints(home_joints)) return false;
    return true;
}

// 抓取物体位姿获取回调函数
void ManipulatorExecutor::objectPoseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    // 在回调函数中只关心被抓取的物体，只从中获取该物体的位姿信息
    const std::string objectName = "tall_cylinder_object";
    for (const auto& transform : msg->transforms) {
        if (transform.child_frame_id == objectName) {
            // 存储最新的物体位姿数据
            {
                std::lock_guard<std::mutex> lock(object_pose_mutex_);
                latest_object_pose_.objectPose.x = transform.transform.translation.x;
                latest_object_pose_.objectPose.y = transform.transform.translation.y;
                latest_object_pose_.objectPose.z = transform.transform.translation.z;
                latest_object_pose_.objectStamp = node_->now();
                has_object_pose_ = true;
            }
        }
    }
}

// 获取被抓取物体最新的位姿数据
bool ManipulatorExecutor::getObjectLatestPose(geometry_msgs::msg::Point & object_pose) {
    {
        std::lock_guard<std::mutex> lock(object_pose_mutex_);

        // 1. 如果还没有从回调函数中获取物体位姿数据，报错
        if (!has_object_pose_) {
            RCLCPP_ERROR(node_->get_logger(), "Object pose not received yet!");
            return false;
        }

        // 返回物体最新的位姿数据
        object_pose = latest_object_pose_.objectPose;
    }
    
    return true;
}

// 夹爪位姿信息回调函数
void ManipulatorExecutor::fingerPoseCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 在回调函数中只关心 panda_finger_joint1，因为两个指关节是 minic 关系
    const std::string finger_name = "panda_finger_joint1";
    const size_t data_len = std::min(msg->name.size(), msg->position.size());

    for (size_t i = 0; i < data_len; i++) {
        if (msg->name[i] == finger_name) {
            {
                std::lock_guard<std::mutex> lock(finger_pose_mutex_);
                latest_finger_pose_.fingerPose = msg->position[i];
                latest_finger_pose_.fingerStamp = node_->now();
                has_finger_joint_ = true;
            }
        }
    } 
}

// 获取夹爪最新的位姿数据
bool ManipulatorExecutor::getFingerLatestPose(double & finger_pose) {
    {
        std::lock_guard<std::mutex> lock(finger_pose_mutex_);

        // 1. 如果还没有从回调函数中获取夹爪数据，报错
        if (!has_finger_joint_) {
            RCLCPP_ERROR(node_->get_logger(), "Finger pose not received yet!");
            return false;
        }

         // 2. 如果时间戳差距过大，认为数据不准确，报错
        if ((node_->now() - latest_finger_pose_.fingerStamp).seconds() > 0.5) {
            RCLCPP_ERROR(node_->get_logger(), "Finger pose has expired!");
            return false;
        }

        // 返回夹爪最新的位姿数据
        finger_pose = latest_finger_pose_.fingerPose;
    }
    
    return true;
}