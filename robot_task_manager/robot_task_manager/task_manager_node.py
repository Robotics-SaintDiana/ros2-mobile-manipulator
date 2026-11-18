import json
import os
import rclpy
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from robot_manipulation_msgs.action import ExecuteBTTask
from ament_index_python.packages import get_package_share_directory


class TaskManagerNode(Node):
    """读取并解析任务配置文件和路点配置文件，然后启动 BT Action"""

    def __init__(self):
        super().__init__('task_manager_node')

        # 获取 robot_task_manager 包的共享目录路径
        share = get_package_share_directory('robot_task_manager')

        # pick_and_place.json 是 LLM 将自然语言转换成的任务文件
        # waypoints.json 是整个导航过程中的所有路径点
        self.declare_parameter('task_file', os.path.join(share, 'tasks', 'pick_and_place.json'))
        self.declare_parameter('waypoints_file', os.path.join(share, 'config', 'waypoints.json'))
        task_path = self.get_parameter('task_file').value
        waypoint_path = self.get_parameter('waypoints_file').value

        # 打开并加载这两个 JSON 文件
        with open(task_path, 'r', encoding='utf-8') as f:
            self.task = json.load(f)
        with open(waypoint_path, 'r', encoding='utf-8') as f:
            self.waypoints = json.load(f)

        # 任务类型只支持 pick_and_place
        if self.task.get('action') != 'pick_and_place':
            raise ValueError("task JSON action must be 'pick_and_place'")

        # 获取路径点的 frame_id，默认为 map
        self.frame_id = self.waypoints.get('frame_id', 'map')

        # 路径点必须包括 pick_station（抓取工位）和 place_station（放置工位）
        # 每个经停点必须包括二维位置坐标(x, y)以及在经停点的位置方向 yaw，否则无法执行
        self.stations = {}
        self.stations_names = ['pick_station', 'place_station']

        for station_name in self.stations_names:
            if station_name not in self.waypoints:
                raise ValueError(f'waypoints JSON missing {station_name}')

            station_data = self.waypoints[station_name]
            for key in ('x', 'y', 'yaw'):
                if key not in station_data:
                    raise ValueError(f'{station_name} missing {key}')

            self.stations[station_name] = {
                'x': float(station_data['x']),
                'y': float(station_data['y']),
                'yaw': float(station_data['yaw'])
            }
        
        # 获取需要抓取的物体以及需要放置在哪个颜色的托盘里面
        self.grasp_object_id = self.task.get('object', {}).get('id', 'irregular_object')
        self.plate_color = self.task.get('destination', {}).get('color', 'red')

        # 创建 BT 的客户端
        self.bt_client = ActionClient(self, ExecuteBTTask, '/manipulator/execute_bt_task')
        self.bt_sent = False
        self.bt_timer = self.create_timer(0.5, self.try_send_bt_goal)

        # 构造抓取位置和放置位置的坐标点
        self.pick_position = self.generate_navigation_pose('pick_station')
        self.place_position = self.generate_navigation_pose('place_station')

    def generate_navigation_pose(self, station_name : str):
        # 获取抓取点或者放置点的目标位置
        station_position = self.stations[station_name]

        # 构造目标点的位置数据
        target_position = PoseStamped()
        target_position.header.frame_id = "map";
        target_position.header.stamp = self.get_clock().now().to_msg()
        target_position.pose.position.x = station_position['x']
        target_position.pose.position.y = station_position['y']
        target_position.pose.position.z = 0.0

        target_yaw = station_position['yaw']
        target_position.pose.orientation.x = 0.0
        target_position.pose.orientation.y = 0.0
        target_position.pose.orientation.z = math.sin(target_yaw / 2.0)
        target_position.pose.orientation.w = math.cos(target_yaw / 2.0)

        return target_position

    def try_send_bt_goal(self):
        # 如果已经执行了 executor_bt 则不再执行
        if self.bt_sent:
            return 

        # BT 客户端没有准备好也不予执行
        if not self.bt_client.server_is_ready():
            return
        
        self.bt_sent = True
        self.bt_timer.cancel()
        self.executor_bt()
        
    def executor_bt(self):
        # 组织下发给 BT 客户端的位姿数据以及托盘颜色
        goal = ExecuteBTTask.Goal()
        goal.pick_position = self.pick_position
        goal.place_position = self.place_position
        goal.plate_color = self.plate_color

        future = self.bt_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"BT stage: {feedback.stage}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('BT task rejected')
            return

        self.get_logger().info('BT task accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        wrapped = future.result()
        result = wrapped.result
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(f'BT task succeeded: {result.message}')
        else:
            self.get_logger().error(
                f'BT task failed: status={wrapped.status}, '
                f'code={result.error_code}, message={result.message}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TaskManagerNode()
        rclpy.spin(node)
    except Exception as exc:
        print(f'[task_reader_node] startup/runtime error: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
