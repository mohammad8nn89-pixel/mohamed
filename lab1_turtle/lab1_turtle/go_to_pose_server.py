"""goToPose action server + speed-limit service + speed parameters."""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits
from lab1_turtle.controller import PoseController

WORLD_MIN, WORLD_MAX = 0.0, 11.0


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.cb_group = ReentrantCallbackGroup()

        # ---- parameters (R7) ----
        self.declare_parameter(
            'max_linear_speed', 2.0,
            ParameterDescriptor(description='Max linear speed [units/s]'))
        self.declare_parameter(
            'max_angular_speed', 2.0,
            ParameterDescriptor(description='Max angular speed [rad/s]'))
        self.add_on_set_parameters_callback(self._validate_params)

        # ---- pub / sub ----
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10,
                                 callback_group=self.cb_group)
        self._pose = None
        self._pose_lock = threading.Lock()
        self._busy = False

        # ---- service to change the limits live ----
        self.create_service(SetSpeedLimits, 'set_speed_limits',
                            self._set_limits_cb, callback_group=self.cb_group)

        # ---- action server (R3) ----
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group)

        self.get_logger().info('goToPose server ready (action: go_to_pose)')

    # ---------- parameters ----------
    @staticmethod
    def _validate_params(params):
        for p in params:
            if p.name in ('max_linear_speed', 'max_angular_speed'):
                if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    return SetParametersResult(
                        successful=False, reason=f'{p.name} must be a number')
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason=f'{p.name} must be > 0')
        return SetParametersResult(successful=True)

    def _set_limits_cb(self, request, response):
        # validate both first so we never apply only one of them
        for name, val in (('max_linear_speed', request.max_linear_speed),
                          ('max_angular_speed', request.max_angular_speed)):
            if not val > 0.0:
                response.success = False
                response.message = f'Rejected: {name} must be > 0 (got {val})'
                self.get_logger().warn(response.message)
                return response

        results = self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE,
                      float(request.max_linear_speed)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE,
                      float(request.max_angular_speed)),
        ])
        ok = all(r.successful for r in results)
        response.success = ok
        if ok:
            response.message = (f'Limits set: linear={request.max_linear_speed}, '
                                f'angular={request.max_angular_speed}')
        else:
            reasons = [r.reason for r in results if not r.successful]
            response.message = 'Rejected: ' + '; '.join(reasons)
        self.get_logger().info(response.message)
        return response

    # ---------- pose ----------
    def _pose_cb(self, msg):
        with self._pose_lock:
            self._pose = (msg.x, msg.y, msg.theta)

    def _get_pose(self):
        with self._pose_lock:
            return self._pose

    # ---------- action ----------
    def _goal_cb(self, goal_request):
        t = goal_request.target
        if self._get_pose() is None:
            self.get_logger().warn('Rejected: no pose received (is turtlesim running?)')
            return GoalResponse.REJECT
        if not (WORLD_MIN < t.x < WORLD_MAX and WORLD_MIN < t.y < WORLD_MAX):
            self.get_logger().warn(
                f'Rejected: target ({t.x:.2f},{t.y:.2f}) outside window (0..11)')
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn('Rejected: another goal is running')
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        try:
            t = goal_handle.request.target
            self.get_logger().info(f'Goal: x={t.x:.2f} y={t.y:.2f} yaw={t.yaw:.2f}')
            ctrl = PoseController(t.x, t.y, t.yaw)
            feedback = GoToPose.Feedback()
            result = GoToPose.Result()
            period = 0.02

            while rclpy.ok():
                x, y, th = self._get_pose()

                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.final_distance = float(ctrl.distance(x, y))
                    result.message = 'Canceled'
                    return result

                # read parameters EVERY cycle -> live changes take effect
                max_lin = self.get_parameter('max_linear_speed').value
                max_ang = self.get_parameter('max_angular_speed').value

                lin, ang = ctrl.step(x, y, th, max_lin, max_ang)
                if ctrl.done:
                    break

                cmd = Twist()
                cmd.linear.x = float(lin)
                cmd.angular.z = float(ang)
                self.cmd_pub.publish(cmd)

                feedback.distance_remaining = float(ctrl.distance(x, y))
                goal_handle.publish_feedback(feedback)
                time.sleep(period)

            self._stop()
            x, y, _ = self._get_pose()
            result.success = True
            result.final_distance = float(ctrl.distance(x, y))
            result.message = 'Reached target pose'
            goal_handle.succeed()
            self.get_logger().info(f'Done (error {result.final_distance:.3f})')
            return result
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
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
