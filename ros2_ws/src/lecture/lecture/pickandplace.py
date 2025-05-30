import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl
import time

class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place_demo')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self.cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SuctionCupControl.Request()

        self.tasks_list = [
            ["move", [106.98, 183.12, 23.97, 0.0], 1],

            # ["move", [138.80, 212.49, -58.86, 0.0], 1],
            # ["gripper", True],
            # ["move", [106.98, 183.12, 23.97, 0.0], 1],
            # ["gripper", False],

            # ["move", [135.66, 151.69, -59.21, 0.0], 1],
            # ["gripper", True],
            # ["move", [106.98, 183.12, 23.97, 0.0], 1],
            # ["gripper", False],

            # ["move", [100.27, 212.73, -58.79, 0.0], 1],
            # ["gripper", True],
            # ["move", [106.98, 183.12, 23.97, 0.0], 1],
            # ["gripper", False],

            # ["move", [93.22, 146.99, -58.62, 0.0], 1],
            # ["gripper", True],
            # ["move", [106.98, 183.12, 23.97, 0.0], 1],
            # ["gripper", False],

            ["move", [60.40, 211.41, -56.03, 0.0], 1],
            ["gripper", True],
            ["move", [106.98, 183.12, 23.97, 0.0], 1],
            ["gripper", False],

            ["move", [62.68, 157.07, -58.66, 0.0], 1],
            ["gripper", True],
            ["move", [106.98, 183.12, 23.97, 0.0], 1],
            ["gripper", False],




            
            # ["move", [106.98, 183.12, 23.97, 0.0], 1],
            # ["move", [156.99, 52.91, 38.41, 0.0], 1],
            # ["move", [155.23, 39.75, -2.92, 0.0], 1],
            
            # ["move", [156.99, 52.91, 38.41, 0.0], 1],
        ]

        
        # self.rate = node.create_rate(1)
        self.goal_num = 0

    def execute(self):
        # time.sleep(1)
        if self.goal_num > len(self.tasks_list)-1:
            rclpy.shutdown()
            sys.exit()
        else:
            self.get_logger().info('*** TASK NUM ***: {0}'.format(self.goal_num))

        if self.tasks_list[self.goal_num][0] == "gripper":
            print('gripper')
            self.send_request(*self.tasks_list[self.goal_num][1:])
            self.timer = self.create_timer(0.1, self.timer_callback, callback_group=ReentrantCallbackGroup())
            self.goal_num = self.goal_num + 1
            
        elif self.tasks_list[self.goal_num][0] == "move":
            self.send_goal(*self.tasks_list[self.goal_num][1:])
            self.goal_num = self.goal_num + 1

    def timer_callback(self):
        if self.srv_future.done():
            result = self.srv_future.result()
            self.get_logger().info('Result of service call: {0}'.format(result))
            self.timer.cancel()
            self.execute()

    def send_request(self, enable_suction):
        self.req.enable_suction = enable_suction
        self.srv_future = self.cli.call_async(self.req)
        
    def send_goal(self, _target, _type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result of action call: {0}'.format(result))
            self.execute()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = PickAndPlace()

    action_client.execute()

    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()


