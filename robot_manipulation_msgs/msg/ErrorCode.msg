# 通用错误码
int32 SUCCESS =  0                        # 任务成功
int32 CANCEL  =  1                        # 任务取消
int32 INTERNAL = 2                        # 意料之外的失败
int32 SKILL_FAILED = 3                    # 简单技能通用失败(回 home / 清理场景等)

# 相机感知阶段错误码
int32 PERCEPTION_NO_DATA     = 100        # 没有点云
int32 PERCEPTION_NO_OBJECT   = 101        # 点云识别后没有物体

# GraspNet 推理阶段错误码
int32 GRASPNET_SERVER_UNAVAILABLE = 200   # 无法与 GraspNet 推理服务器进行正常交互
int32 GRASPNET_TIMEOUT            = 201   # GraspNet 推理超时
int32 GRASPNET_NO_CANDIDATES      = 202   # GraspNet 推理后没有获得候选抓取位姿
int32 GRASPNET_FAILED             = 203   # GraspNet 推理失败

# 候选抓取位姿过滤阶段错误码
int32 FILTER_NO_FEASIBLE = 300            # 过滤后没有符合要求的候选位姿

# 抓取物体阶段错误码(Pick 阶段)
int32 PICK_PLAN_FAILED       = 400        # 抓取过程中路径规划失败
int32 PICK_GRASP_FAILED      = 401        # 抓取过程中夹爪没有抓取到物体
int32 PICK_LIFT_CHECK_FAILED = 402        # 抓取后检测到物体没有正确抬升  

# 放置物体阶段错误码(Place 阶段)
int32 PLACE_PLAN_FAILED = 500             # 放置过程中路径规划失败

# 其他错误码
int32 SCENE_PREPARE_FAILED = 601          # 向规划场景中增加物体失败
int32 SCENE_REMOVE_FAILED  = 602          # 向规划场景中删除物体失败