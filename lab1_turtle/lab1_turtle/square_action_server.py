"""Square action server: reset, then exactly 4 goToPose goals."""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from lab1_interfaces.action import DrawSquare, GoToPose
from lab1_interfaces.msg import TargetPose
from lab1_turtle.reset_service import START_X, START_Y

MAX_SIDE = 4.5   # keeps the square inside the 0..11 window


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')
        grp = ReentrantCallbackGroup()
        self.reset_cli = self.create_client(Trigger, 'reset_turtle', callback_group=grp)
        self.gtp_cli = ActionClient(self, GoToPose, 'go_to_pose', callback_group=grp)
        self._busy = False
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=grp)
        self.get_logger().info('Square action server ready (action: draw_square)')

    def _goal_cb(self, goal):
        if not (0.0 < goal.side_length <= MAX_SIDE):
            self.get_logger().warn(f'Rejected: side_length must be in (0, {MAX_SIDE}]')
            return GoalResponse.REJECT
        if self._busy:
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _wait(fut, timeout=120.0):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.01)
        return True

    def _reset(self):
        if not self.reset_cli.wait_for_service(timeout_sec=5.0):
            return False
        fut = self.reset_cli.call_async(Trigger.Request())
        return self._wait(fut, 10.0) and fut.result().success

    def _go_to(self, x, y, yaw):
        goal = GoToPose.Goal()
        goal.target = TargetPose(x=float(x), y=float(y), yaw=float(yaw))
        fut = self.gtp_cli.send_goal_async(goal)
        if not self._wait(fut, 10.0):
            return False
        handle = fut.result()
        if not handle.accepted:
            return False
        res_fut = handle.get_result_async()
        if not self._wait(res_fut):
            return False
        return res_fut.result().result.success

    def _execute(self, goal_handle):
        try:
            s = goal_handle.request.side_length
            fb = DrawSquare.Feedback()
            result = DrawSquare.Result()

            if not self.gtp_cli.wait_for_server(timeout_sec=5.0):
                goal_handle.abort()
                result.success, result.message = False, 'go_to_pose server not available'
                return result

            fb.current_side, fb.status = 0, 'Resetting'
            goal_handle.publish_feedback(fb)
            if not self._reset():
                goal_handle.abort()
                result.success, result.message = False, 'Reset failed'
                return result

            x0, y0 = START_X, START_Y
            # each corner + the heading needed to start the NEXT side
            corners = [
                (x0 + s, y0,     math.pi / 2),
                (x0 + s, y0 + s, math.pi),
                (x0,     y0 + s, -math.pi / 2),
                (x0,     y0,     0.0),
            ]
            for i, (cx, cy, yaw) in enumerate(corners, start=1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success, result.message = False, 'Canceled'
                    return result
                fb.current_side, fb.status = i, f'Drawing side {i}/4'
                goal_handle.publish_feedback(fb)
                if not self._go_to(cx, cy, yaw):
                    goal_handle.abort()
                    result.success, result.message = False, f'Side {i} failed'
                    return result

            goal_handle.succeed()
            result.success, result.message = True, 'Square completed'
            return result
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = SquareActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
