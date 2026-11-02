import os
import sys
import struct
import numpy as np
import torch
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse

from robot_manipulation_msgs.action import InferGrasps
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

# ============================================================
# 1. 配置 GraspNet-baseline 路径
# ============================================================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
GRASPNET_ROOT = os.path.join(CURRENT_DIR, "algorithms", "graspnet_impl", "graspnet_baseline")

MODELS_DIR = os.path.join(GRASPNET_ROOT, "models")
UTILS_DIR = os.path.join(GRASPNET_ROOT, "utils")
DATASET_DIR = os.path.join(GRASPNET_ROOT, "dataset")

# 将官方代码目录添加到 Python 搜索路径
for path in [MODELS_DIR, UTILS_DIR, DATASET_DIR, GRASPNET_ROOT]:
    if path not in sys.path:
        sys.path.insert(0, path)

# ============================================================
# 2. 导入 GraspNet 官方模块
# ============================================================
try:
    from graspnet import GraspNet, pred_decode
    from collision_detector import ModelFreeCollisionDetector
    from graspnetAPI import GraspGroup
except ImportError as e:
    raise ImportError(f"导入 GraspNet 官方模块失败，请检查相关依赖和路径：{e}") from e


# ============================================================
# 3. GraspNet ROS 2 节点
# ============================================================
class GraspNetNode(Node):

    def __init__(self):
        super().__init__("graspnet_executor_node")

        # 3.1 声明参数
        default_checkpoint = os.path.join(GRASPNET_ROOT, "checkpoint", "checkpoint-rs.tar")
        self.declare_parameter("checkpoint_path", default_checkpoint)
        self.declare_parameter("num_point", 20000)
        self.declare_parameter("collision_thresh", 0.01)
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("depth_offset_scale", 1.0)

        # ---- 抓取后处理相关参数 ----
        # 碰撞检测前先按分数截断到多少个候选：碰撞是"候选数 x 点数"的计算，全量跑很慢很占内存
        self.declare_parameter("pre_collision_top", 200)
        # 相对分数门槛：score 不是概率、可大于 1，整体水平还会随点云/视角漂移，
        # 所以只能用"同一帧内相对最高分的比例"，不能用绝对阈值。0 表示关闭
        self.declare_parameter("score_ratio", 0.4)
        # 进刀/退让走廊要留多长，碰撞检测用它检查"手伸过去、退出来的路上有没有东西"
        self.declare_parameter("approach_dist", 0.05)
        # 空抓检测：两指之间几乎没有点 -> 抓了个寂寞，直接丢掉
        self.declare_parameter("enable_empty_check", True)
        self.declare_parameter("empty_thresh", 0.02)
        # 最终返回给机械臂的候选个数（C++ 侧会按顺序逐个尝试）
        self.declare_parameter("keep_top_k", 5)
        # 可选的"向下接近"预筛：0 表示关闭；0.6 表示要求 approach 与竖直向下夹角 <= 53 度。
        # 它只是便宜的近似，真正的判据是含桌面的碰撞检测，别指望它代替后者
        self.declare_parameter("min_approach_down", 0.0)

        checkpoint_path = self.get_parameter("checkpoint_path").value
        self.collision_thresh = float(self.get_parameter("collision_thresh").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.depth_offset_scale = float(self.get_parameter("depth_offset_scale").value)
        self.pre_collision_top = int(self.get_parameter("pre_collision_top").value)
        self.score_ratio = float(self.get_parameter("score_ratio").value)
        self.approach_dist = float(self.get_parameter("approach_dist").value)
        self.enable_empty_check = bool(self.get_parameter("enable_empty_check").value)
        self.empty_thresh = float(self.get_parameter("empty_thresh").value)
        self.keep_top_k = int(self.get_parameter("keep_top_k").value)
        self.min_approach_down = float(self.get_parameter("min_approach_down").value)

        # 3.2 初始化计算设备
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"正在使用设备进行推理: {self.device}")

        # 3.3 初始化 GraspNet 并加载权重
        self.net = self.init_graspnet_model(checkpoint_path)

        # 3.4 初始化 Graspnet 感知节点服务端
        self.action_server = ActionServer(
            self,
            InferGrasps,
            "/graspnet/infer_grasps",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    # ========================================================
    # 4. 初始化 GraspNet 模型
    # ========================================================
    def init_graspnet_model(self, ckpt_path):
        """初始化 GraspNet 并加载预训练权重。"""
        net = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False
        )
        net.to(self.device)

        if not os.path.isfile(ckpt_path):
            raise FileNotFoundError(f"未找到 GraspNet 权重文件：{ckpt_path}")

        checkpoint = torch.load(ckpt_path, map_location=self.device)

        if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
            state_dict = checkpoint["model_state_dict"]
        else:
            state_dict = checkpoint

        net.load_state_dict(state_dict)
        net.eval()
        self.get_logger().info(f"成功加载预训练权重: {ckpt_path}")

        return net

    # ========================================================
    # 5. 根据输入的点云，调用 Graspnet 获取排名前 10 的抓取位姿
    # ========================================================
    def infer_grasps(self, msg: PointCloud2):
        try:
            # 5.1 读取 XYZ 和 RGB 字段
            cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
            if len(cloud_data) < 50:
                self.get_logger().warn("收到的目标点云数量过少，无法进行抓取推理！")
                return [], []

            points = []
            colors = []
            for p in cloud_data:
                points.append([p[0], p[1], p[2]])
                try:
                    rgb_float = p[3]
                    packed_int = struct.unpack('<I', struct.pack('<f', rgb_float))[0]
                    r = (packed_int >> 16) & 0x00FF
                    g = (packed_int >> 8) & 0x00FF
                    b = packed_int & 0x00FF
                    colors.append([r / 255.0, g / 255.0, b / 255.0])
                except Exception:
                    colors.append([0.5, 0.5, 0.5])

            cloud_np = np.array(points, dtype=np.float32)
            colors_np = np.array(colors, dtype=np.float64)

            valid_mask = np.isfinite(cloud_np).all(axis=1)
            cloud_np = cloud_np[valid_mask]
            colors_np = colors_np[valid_mask]

            if len(cloud_np) < 50:
                self.get_logger().warn("过滤无效坐标后，剩余点云数量不足！")
                return [], []

            cloud_sampled = cloud_np  

            # 5.3 构造网络输入 (Batch size = 1, Shape: [1, N, 3])
            cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis, ...]).to(self.device)
            end_points = {"point_clouds": cloud_tensor}

            # 5.4 GraspNet 前向推理
            with torch.no_grad():
                end_points = self.net(end_points)
                grasp_preds = pred_decode(end_points)

            if len(grasp_preds) == 0:
                self.get_logger().warn("GraspNet 未返回抓取候选结果！")
                return [], []

            gg_array = grasp_preds[0].detach().cpu().numpy()
            gg = GraspGroup(gg_array)

            if len(gg) == 0:
                self.get_logger().warn("GraspNet 没有生成有效抓取候选！")
                return [], []

            # 5.5 抓取候选去重与排序
            gg.nms()
            gg.sort_by_score()

            if len(gg) == 0:
                self.get_logger().warn("抓取候选经过 NMS 处理后为空！")
                return [], []

            # 5.6 碰撞检测
            if self.collision_thresh > 0:
                mfcdetector = ModelFreeCollisionDetector(cloud_np, voxel_size=self.voxel_size)
                collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
                gg = gg[~collision_mask]

            if len(gg) == 0:
                self.get_logger().warn("经过碰撞检测后，未找到任何可行的抓取位姿！")
                return [], []

            # 抓取候选后处理
            n_network = len(gg)
            gg.sort_by_score()     # 根据抓取质量分数从高到低进行降序排序

            # 只保留分数最高的前 pre_collision_top 个候选
            gg = gg[: self.pre_collision_top]
            n_top = len(gg)
            max_score = float(gg.scores[0]) if n_top > 0 else 0.0

            # 在所有保留的候选中，继续筛选分数高于当前最高分 50% 的候选位姿
            if (self.score_ratio > 0.0 and n_top > 0):
                gg = gg[gg.scores >= self.score_ratio * max_score]
            n_score = len(gg)

            # 剔除可能导致碰撞，或者空抓的位姿点
            if self.collision_thresh > 0.0:
                # 初始化无模型碰撞检测器开始碰撞检测
                mfcdetector = ModelFreeCollisionDetector(cloud_np, voxel_size=self.voxel_size)

                # 开启空抓检测
                if self.enable_empty_check:
                    collision_mask, empty_mask = mfcdetector.detect(
                        gg,
                        approach_dist=self.approach_dist,
                        collision_thresh=self.collision_thresh,
                        return_empty_grasp=True,
                        empty_thresh=self.empty_thresh,
                    )
                    gg = gg[~(collision_mask | empty_mask)]
                else:
                    collision_mask = mfcdetector.detect(
                        gg,
                        approach_dist=self.approach_dist,
                        collision_thresh=self.collision_thresh,
                    )
                    gg = gg[~collision_mask]
            n_collision = len(gg)

            if n_collision == 0:
                self.get_logger().warn(
                    f"过滤后没有可行候选：网络 {n_network} -> 分数前{self.pre_collision_top} {n_top}"
                    f" -> 分数门槛 {n_score} -> 碰撞/空抓 0"
                )
                return [], []

            # NMS 去重
            gg = gg.nms()
            gg.sort_by_score()
            n_nms = len(gg)

            self.get_logger().info(
                f"候选过滤: 网络 {n_network} -> 分数前{self.pre_collision_top} {n_top}"
                f" -> 分数门槛 {n_score} -> 碰撞/空抓 {n_collision} -> NMS {n_nms}"
            )

            # --------------------------------------------------------------
            try:
                o3d_cloud = o3d.geometry.PointCloud()
                o3d_cloud.points = o3d.utility.Vector3dVector(cloud_np)

                # 设置点云颜色
                if len(colors_np) == len(cloud_np):
                    o3d_cloud.colors = o3d.utility.Vector3dVector(colors_np)

                if len(gg) == 0:
                    self.get_logger().warn(
                        "GraspNet 没有检测到有效抓取，跳过 Open3D 可视化"
                    )
                else:

                    # 获取前 5 个抓取（如果总数小于 5，Python 的切片会自动取到最大可用数量）
                    num_to_show = min(5, len(gg))
                    top_grasps = gg[0:num_to_show]

                    self.get_logger().info(
                        f"检测到有效抓取，将可视化前 {num_to_show} 个最高分抓取位置："
                    )
                    for i in range(num_to_show):
                        self.get_logger().info(
                            f"  第 {i+1} 名抓取得分: {gg[i].score:.4f}"
                        )

                    # 转换为 Open3D 几何体列表（to_open3d_geometry_list() 会把整个 GraspGroup 转为包含多个夹爪几何体的列表）
                    top_grasps_geometry = top_grasps.to_open3d_geometry_list()

                    coord_frame = (
                        o3d.geometry.TriangleMesh.create_coordinate_frame(
                            size=0.1,
                            origin=[0, 0, 0]
                        )
                    )

                    self.get_logger().info(
                        f"正在打开 Open3D 窗口显示前 {num_to_show} 个抓取位姿..."
                    )

                    o3d.visualization.draw_geometries(
                        [
                            o3d_cloud,
                            *top_grasps_geometry, # 用解包符号将多个夹爪几何体全部传入
                            coord_frame
                        ],
                        window_name="GraspNet Top 5 Grasps Visualization",
                        width=1280,
                        height=720
                    )

            except Exception as viz_err:
                self.get_logger().warn(
                    f"Open3D 可视化过程跳过或出错: {viz_err}"
                )
            # --------------------------------------------------------------

            # 获取评分最高的前 10 个抓取候选并构造齐次变换矩阵
            # 最多返回 10 个抓取候选位姿，不足 10 个的时候，有多少返回多少
            top_k = min(self.keep_top_k, len(gg))

            # 函数最终会返回抓取候选位姿，以及相应的得分
            grasp_poses = []
            scores = []

            # 遍历 Graspnet 推理得到的结果，组装数据并返回
            for i in range(top_k):
                curr_grasp = gg[i]

                # 构造 4x4 抓取位姿矩阵
                pose_matrix = np.eye(4, dtype=np.float64)
                pose_matrix[:3, :3] = curr_grasp.rotation_matrix

                grasp_depth = float(np.clip(curr_grasp.depth, 0.0, 0.04))
                depth_offset = self.depth_offset_scale * grasp_depth
                approach_dir = pose_matrix[:3, 0]
                pose_matrix[:3, 3] = curr_grasp.translation + depth_offset * approach_dir

                # 转换为 ROS PoseStamped
                target_pose = PoseStamped()
                target_pose.header = msg.header
                target_pose.pose.position.x = float(pose_matrix[0, 3])
                target_pose.pose.position.y = float(pose_matrix[1, 3])
                target_pose.pose.position.z = float(pose_matrix[2, 3])

                rotation = R.from_matrix(pose_matrix[:3, :3])
                quat = rotation.as_quat()
                target_pose.pose.orientation.x = float(quat[0])
                target_pose.pose.orientation.y = float(quat[1])
                target_pose.pose.orientation.z = float(quat[2])
                target_pose.pose.orientation.w = float(quat[3])

                # 收集位姿和对应的评分
                grasp_poses.append(target_pose)
                scores.append(float(curr_grasp.score))

            return grasp_poses, scores

        except Exception as e:
            self.get_logger().error(f"处理目标点云时发生异常：{type(e).__name__}: {e}")
            self.get_logger().error("详细错误信息：\n" + __import__("traceback").format_exc())
            return [], []

    # ========================================================
    # 6. Graspnet 服务端的回调函数
    # ========================================================
    def goal_callback(self, goal_request):
        """决定是否接受新的推理请求。"""
        self.get_logger().info("收到 GraspNet 推理请求")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理客户端的取消请求。"""
        self.get_logger().info("收到取消推理请求")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """接收点云，执行 GraspNet 推理，并返回候选抓取位姿。"""
        result = InferGrasps.Result()
        try:
            # 1. 获取客户端发送的点云，并对其合法性进行判断
            cloud_msg = goal_handle.request.cloud

            if cloud_msg.width == 0 or cloud_msg.height == 0:
                result.success = False
                result.message = "Input point cloud is empty!"
                result.grasp_poses = []
                result.scores = []

                goal_handle.abort()
                return result

            # 2. 调用 Graspnet 对传入的点云数据进行推理
            grasp_poses, scores = self.infer_grasps(cloud_msg)

            # 如果推理失败，返回的 grasp_poses, scores 均为 []
            if not grasp_poses:
                result.success = False
                result.message = "No valid grasp candidates found!"
                result.grasp_poses = []
                result.scores = []

                goal_handle.abort()
                return result

            # 如果返回的 grasp_poses 与 scores 长度不一致，也是一种错误情况
            if len(grasp_poses) != len(scores):
                result.success = False
                result.message = "Mismatch between the number of grasp poses and scores!"
                result.grasp_poses = []
                result.scores = []

                goal_handle.abort()
                return result

            # 3. 将推理结果返回
            result.success = True
            result.message = (f"Inference successful: {len(grasp_poses)} grasp candidates generated")
            result.grasp_poses = grasp_poses
            result.scores = scores

            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"GraspNet Action Failed: {type(e).__name__}: {e}")

            result.success = False
            result.message = str(e)
            result.grasp_poses = []
            result.scores = []

            if goal_handle.is_active:
                goal_handle.abort()
            return result

# ============================================================
# 7. ROS 2 主函数
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GraspNetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()