import json
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_llm_task_parser.llm_client import LLMClient, LLMError
from robot_llm_task_parser.prompts import build_system_prompt

class TaskParserNode(Node):
    def __init__(self):
        super().__init__('task_parser_node')

        # 加载节点需要的参数
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        self.declare_parameter('model', 'deepseek-chat')
        self.declare_parameter('api_key_env', 'DEEPSEEK_API_KEY')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('colors', ['red', 'green', 'blue'])

        # waypoints_file 的默认位置在 lauch 节点中定义了，否则需要在终端输入文件地址
        waypoints_file = self.get_parameter('waypoints_file').value
        if not waypoints_file:
            raise RuntimeError('waypoints_file 参数不能为空')
        with open(waypoints_file, 'r', encoding='utf-8') as f:
            self.waypoints = json.load(f)

        # 从文件的站点位置中筛选出包含有 x, y, yaw 信息的有效导航点
        self.station = {
            name for name, info in self.waypoints.items()
            if isinstance(info, dict) and 'x' in info and 'y' in info and 'yaw' in info
        }
        self.colors = list(self.get_parameter('colors').value)

        # 创建 LLM 客户端
        self.LLMClient = LLMClient(
            base_url = self.get_parameter('base_url').value,
            model = self.get_parameter('model').value,
            api_key_env = self.get_parameter('api_key_env').value,
            system_prompt = build_system_prompt(self.waypoints, self.colors),
            temperature = self.get_parameter('temperature').value,
            timeout = self.get_parameter('timeout_sec').value,
            max_attempts = self.get_parameter('max_attempts').value
        )

        # 线程锁与标志位，同一时刻只允许处理一个指令，多余的指令不允许处理
        self._lock = threading.Lock()
        self._busy = False

        # group 参数允许该节点运行时处理最新发送的消息，但后续的处理会立刻拒绝该请求，防止一条请求处理结束后立刻处理下一个请求
        group = ReentrantCallbackGroup()
        self.create_subscription(String, '/task/command', self.on_command, 10, callback_group=group)
        self.plan_pub = self.create_publisher(String, '/task/plan', 10)

    def on_command(self, msg):
        # 先获取用户的输入指令，空指令则不处理
        user_command = msg.data.strip()
        if not user_command:
            return

        # 判断此时刻是否正在处理上一条指令
        with self._lock:
            if self._busy:
                self.get_logger().warn('still parsing previous command, ignored')
                return 

            self._busy = True

        # 尝试将用户指令发送给 LLM 进行解析
        try:
            try:
                task_plan = self.LLMClient.parse(user_command)
            except LLMError as e:
                self.publish_failure(str(e))
                return

            # 如果用户发送的指令无效
            if task_plan.action == 'unknown':
                self.publish_failure(task_plan.reason)
                return

            # 如果 LLM 解析的导航点不是有效的导航点
            if task_plan.object.source not in self.station:
                self.publish_failure(f'unknown pick waypoint: {task_plan.object.source}')
                return
            if task_plan.destination.location not in self.station:
                self.publish_failure(f'unknown place waypoint: {task_plan.destination.location}')
                return

            # 解析成功，向话题发送成功消息
            payload = {'success': True, 'message': 'ok', 'plan': task_plan.model_dump()}
            self.plan_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        finally:
            with self._lock:
                self._busy = False

    def publish_failure(self, message):
        payload = {'success': False, 'message': message, 'plan': None}
        self.plan_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().warn(f'parse failed: {message}')

def main():
    rclpy.init()
    node = TaskParserNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()