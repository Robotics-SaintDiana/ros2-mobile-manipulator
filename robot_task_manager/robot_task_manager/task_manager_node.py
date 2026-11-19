import json
import os
import rclpy
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from robot_manipulation_msgs.action import ExecuteBTTask
from ament_index_python.packages import get_package_share_directory


class TaskManagerNode(Node):
    """从话题或者文件中读取并解析任务配置文件和路点配置文件，然后启动 BT Action"""

    def __init__(self):
        super().__init__('task_manager_node')

        self.declare_parameter('start_from_file', False)

        share = get_package_share_directory('robot_task_manager')
        self.declare_parameter('task_file', os.path.join(share, 'tasks', 'pick_and_place.json'))
        self.declare_parameter('waypoints_file', os.path.join(share, 'config', 'waypoints.json'))

        waypoints_file = self.get_parameter('waypoints_file').value
        with open(waypoints_file, 'r', encoding='utf-8') as f:
            self.waypoints = json.load(f)

        self.frame_id = self.waypoints.get('frame_id', 'map')

        self.stations = {}
        for station_name in ('pick_station', 'place_station'):
            if station_name not in self.waypoints:
                raise ValueError(f'waypoints JSON missing {station_name}')

            station_data = self.waypoints[station_name]
            for key in ('x', 'y', 'yaw'):
                if key not in station_data:
                    raise ValueError(f'{station_name} missing {key}')

            self.stations[station_name] = {
                'x': float(station_data['x']),
                'y': float(station_data['y']),
                'yaw': float(station_data['yaw']),
            }

        self.is_busy = False

        # 创建 BT 的客户端
        self.bt_client = ActionClient(self, ExecuteBTTask, '/manipulator/execute_bt_task')

        if self.get_parameter('start_from_file').value:
            task_path = self.get_parameter('task_file').value
            with open(task_path, 'r', encoding='utf-8') as f:
                self.task = json.load(f)

            if self.task.get('action') != 'pick_and_place':
                raise ValueError("task JSON action must be 'pick_and_place'")

            self.grasp_object_id = self.task.get('object', {}).get('id', 'irregular_object')
            self.plate_color = self.task.get('destination', {}).get('color', 'red')

            self.pick_position = self.generate_navigation_pose('pick_station')
            self.place_position = self.generate_navigation_pose('place_station')

            self.bt_file_sent = False
            self.bt_file_timer = self.create_timer(0.5, self.try_send_bt_file)
        else:
            self.create_subscription(String, '/task/plan', self.parse_command, 10)

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

    def try_send_bt_file(self):
        # 如果已经执行了 executor_bt 则不再执行
        if self.bt_file_sent:
            return 

        # BT 客户端没有准备好也不予执行
        if not self.bt_client.server_is_ready():
            return
        
        self.bt_file_sent = True
        self.bt_file_timer.cancel()
        self.executor_bt()

    def parse_command(self, msg):
        # 如果正在处理上一条指令，不予执行
        if self.is_busy:
            self.get_logger().warn('a task is already running, new plan ignored')
            return

        # 尝试获取话题中的 json 数据
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('task plan is not valid JSON')
            return

        # 如果 LLM 解析出的内容并不是有效的指令
        if not payload.get('success'):
            self.get_logger().warn(f"task parse failed: {payload.get('message', '')}")
            return

        # 尝试根据 LLM 解析出的 json 构建下发给 BT 的指令
        try:
            goal = self.build_goal(payload.get('plan') or {}) 
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f'invalid task plan: {e}')
            return

        # 下发指令给 BT
        self.start_task(goal)

    def build_goal(self, plan):
        # 获取指令中的内容
        pick_location = plan['object']['source']
        place_location = plan['destination']['location']
        plate_color = plan['destination']['color']
        if plate_color not in ('red', 'green', 'blue'):
            raise ValueError(f'unsupported plate color: {plate_color}')

         # 组织下发给 BT 客户端的位姿数据以及托盘颜色
        goal = ExecuteBTTask.Goal()
        goal.pick_position = self.generate_navigation_pose(pick_location)
        goal.place_position = self.generate_navigation_pose(place_location)
        goal.plate_color = plate_color

        return goal

    def start_task(self, goal):
        if not self.bt_client.server_is_ready():
            self.get_logger().warn('BT action server not ready, drop task')
            return

        self.is_busy = True
        future = self.bt_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        
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
            self.is_busy = False
            self.get_logger().error('BT task rejected')
            return

        self.get_logger().info('BT task accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.is_busy = False

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
