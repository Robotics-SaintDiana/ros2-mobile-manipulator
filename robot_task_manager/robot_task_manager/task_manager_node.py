import json
import os
import rclpy
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from robot_manipulation_msgs.action import ManipulateObject
from ament_index_python.packages import get_package_share_directory


class TaskManagerNode(Node):
    """读取并解析任务配置文件和路点配置文件，然后按顺序向导航节点分发导航目标点，并监听导航状态进行状态机跳转"""

    # 整个任务周期中可能处于的任务状态:
    WAIT_NAV_READY = 'WAIT_NAV_READY'
    NAV_TO_PICK = 'NAV_TO_PICK'
    WAIT_MANIPULATOR_READY_FOR_PICK = 'WAIT_MANIPULATOR_READY_FOR_PICK'
    PICKING = 'PICKING'
    WAIT_NAV_READY_FOR_PLACE = 'WAIT_NAV_READY_FOR_PLACE'
    NAV_TO_PLACE = 'NAV_TO_PLACE'
    WAIT_MANIPULATOR_READY_FOR_PLACE = 'WAIT_MANIPULATOR_READY_FOR_PLACE'
    PLACING = 'PLACING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'

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
        self.place_color = self.task.get('destination', {}).get('color', 'red')

        # 创建下发导航任务和机械臂任务的客户端         
        self.navigation_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.manipulator_client = ActionClient(self, ManipulateObject, '/manipulator/execute_task')

        # 接收导航机械臂客户端处理信息的句柄
        self.navigation_goal_handle = None
        self.manipulator_goal_handle = None

        # 任务当前所处的状态,初始值默认为 WAIT_NAV_READY
        self.task_state = self.WAIT_NAV_READY

        # 周期性检查是否可以推进任务状态机
        self.task_timer = self.create_timer(0.5, self.advance_task)

    # 核心函数:根据当前的状态确定下一步指令
    def advance_task(self):
        # 特殊情况:如果当前状态处于 SUCCEEDED 或者 FAILED,表明任务结束,不进行任何处理
        if self.task_state in (self.SUCCEEDED, self.FAILED):
            return

        # 如果机器人处于等待前往抓取工位,且此时导航准备完毕,则发送导航数据
        if self.task_state == self.WAIT_NAV_READY:
            self.send_navigation_goal('pick_station')
            return

        # 如果机器人已经处于抓取工位，正在等待抓取，且此时机械臂准备完毕，则发送机械臂数据
        if self.task_state == self.WAIT_MANIPULATOR_READY_FOR_PICK:
            self.send_manipulator_goal('pick')
            return

        # 如果机器人抓取完物体，正在等待前往放置位置，且此时导航准备完毕，则发送导航数据
        if self.task_state == self.WAIT_NAV_READY_FOR_PLACE:
            self.send_navigation_goal('place_station')
            return

        # 如果机器人已经处于放置工位，正在等待抓取，且此时机械臂准备完毕，则发送机械臂数据
        if self.task_state == self.WAIT_MANIPULATOR_READY_FOR_PLACE:
            self.send_manipulator_goal('place')
            return

    def send_navigation_goal(self, station_name: str):
        # 根据输入的参数更新任务当前状态
        if station_name == 'pick_station':
            target_state = self.NAV_TO_PICK
        elif station_name == 'place_station':
            target_state = self.NAV_TO_PLACE
        else:
            self.task_state == self.FAILED
            self.get_logger().error(f'Unknown station: {station_name}')
            return

         # 若此时服务器没有准备好，可以依靠定时器进行等待
        if not self.navigation_client.server_is_ready():
            self.get_logger().debug('Waiting for navigation Action Server...')
            return

        # 更新当前任务状态(此处更新防止客户端没有启动，但任务状态已经发生了改变)
        self.task_state = target_state

         # 构造 Nav2 目标对象
        navigation_goal = NavigateToPose.Goal()
        navigation_goal.pose = PoseStamped()
        navigation_goal.pose.header.frame_id = self.frame_id
        navigation_goal.pose.header.stamp = self.get_clock().now().to_msg()
        navigation_goal.pose.pose.position.x = self.stations[station_name]['x']
        navigation_goal.pose.pose.position.y = self.stations[station_name]['y']
        navigation_goal.pose.pose.position.z = 0.0
        navigation_goal.pose.pose.orientation.x = 0.0
        navigation_goal.pose.pose.orientation.y = 0.0
        navigation_goal.pose.pose.orientation.z = math.sin(self.stations[station_name]['yaw'] / 2.0)
        navigation_goal.pose.pose.orientation.w = math.cos(self.stations[station_name]['yaw'] / 2.0)

        # 异步发送导航目标
        send_navigation_goal_future = self.navigation_client.send_goal_async(navigation_goal)

        # 处理导航任务是否被接收
        send_navigation_goal_future.add_done_callback(self.navigation_goal_response_callback)

     # 处理导航任务是否被接收
    def navigation_goal_response_callback(self, future):
        # 尝试获取导航任务的接收结果，无法获取则报错
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.task_state = self.FAILED
            self.get_logger().error(f'Failed to send navigation goal: {exc}')
            return

        # 任务未被接收则报错
        if not goal_handle.accepted:
            self.task_state = self.FAILED
            self.get_logger().error('Navigation goal was rejected by the server.')
            return

        # 任务接收则异步接收任务结果，并在回调函数中处理最终的返回结果
        self.navigation_goal_handle = goal_handle
        self.get_logger().info('Navigation goal accepted; waiting for result.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    # 处理导航最终的返回结果
    def navigation_result_callback(self, future):
        # 尝试获取导航任务的最终结果，无法获取则报错
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.task_state = self.FAILED
            self.navigation_goal_handle = None
            self.get_logger().error(f'Failed to receive navigation result: {exc}')
            return

        status = wrapped_result.status

        # 清理当前任务句柄
        self.navigation_goal_handle = None

        # 检查 Action 是否成功，以及导航任务是否成功，有一个不成功都认为是失败
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.task_state = self.FAILED
            self.get_logger().error(
                f'navigation task failed')
            return

        self.get_logger().info(f'navigation task succeeded')

        # 任务开始后先前往抓取位置处
        if self.task_state == self.NAV_TO_PICK:
            self.task_state = self.WAIT_MANIPULATOR_READY_FOR_PICK
            self.advance_task()
            return
        
        # 抓取完物体后前往放置位置处
        if self.task_state == self.NAV_TO_PLACE:
            self.task_state = self.WAIT_MANIPULATOR_READY_FOR_PLACE
            self.get_logger().info('Navigation task completed successfully!')
            self.advance_task()
            return

        # 防止状态机与返回的任务结果不一致
        self.task_state = self.FAILED
        self.get_logger().error('Received manipulator result in an unexpected task state.')

    def send_manipulator_goal(self, action_type: str):
        # 根据输入的参数更新任务当前状态
        if action_type == 'pick':
            target_state = self.PICKING
        elif action_type == 'place':
            target_state = self.PLACING
        else:
            self.get_logger().error(f'Unknown manipulator action: {action_type}')
            return

        # 若此时服务器没有准备好，可以依靠定时器进行等待
        if not self.manipulator_client.server_is_ready():
            self.get_logger().debug('Waiting for manipulator Action Server...')
            return

        # 更新当前任务状态(此处更新防止客户端没有启动，但任务状态已经发生了改变)
        self.task_state = target_state

        # 构造下发的 Action Goal
        manipulator_goal = ManipulateObject.Goal()
        manipulator_goal.action_type = action_type
        manipulator_goal.object_id = self.grasp_object_id
        manipulator_goal.color = self.place_color

        # 异步发送机械臂抓取目标，处理机械臂执行过程中的阶段反馈
        send_manipulator_goal_future = self.manipulator_client.send_goal_async(manipulator_goal)

        # 处理机械臂任务是否被接收
        send_manipulator_goal_future.add_done_callback(self.manipulator_goal_response_callback)

    # 处理机械臂任务是否被接收
    def manipulator_goal_response_callback(self, future):
        # 尝试获取机械臂任务的接收结果，无法获取则报错
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.task_state = self.FAILED
            self.get_logger().error(f'Failed to send manipulator goal: {exc}')
            return

        # 任务未被接收则报错
        if not goal_handle.accepted:
            self.task_state = self.FAILED
            self.get_logger().error('Manipulator goal was rejected by the server.')
            return

        # 任务接收则异步接收任务结果，并在回调函数中处理最终的返回结果
        self.manipulator_goal_handle = goal_handle
        self.get_logger().info('Manipulator goal accepted; waiting for result.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.manipulator_result_callback)

    # 处理机械臂最终的返回结果
    def manipulator_result_callback(self, future):
        # 尝试获取机械臂任务的最终结果，无法获取则报错
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.task_state = self.FAILED
            self.manipulator_goal_handle = None
            self.get_logger().error(f'Failed to receive manipulator result: {exc}')
            return

        result = wrapped_result.result
        status = wrapped_result.status

        # 清理当前任务句柄
        self.manipulator_goal_handle = None

        # 检查 Action 是否成功，以及机械臂任务是否成功，有一个不成功都认为是失败
        if (status != GoalStatus.STATUS_SUCCEEDED or not result.success):
            self.task_state = self.FAILED
            self.get_logger().error(
                f'Manipulator task failed: '
                f'status={status}, message={result.message}'
            )
            return

        self.get_logger().info(
            f'Manipulator task succeeded: {result.message}'
        )

        # 抓取成功后，等待导航前往放置工位
        if self.task_state == self.PICKING:
            self.task_state = self.WAIT_NAV_READY_FOR_PLACE
            self.advance_task()
            return

        # 放置成功后，整个任务完成
        if self.task_state == self.PLACING:
            self.task_state = self.SUCCEEDED
            self.get_logger().info('Pick-and-place task completed successfully!')
            return

        # 防止状态机与返回的任务结果不一致
        self.task_state = self.FAILED
        self.get_logger().error('Received manipulator result in an unexpected task state.')

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
