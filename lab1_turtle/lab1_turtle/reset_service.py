"""Reset service: clears the screen and teleports the turtle to its start pose.

Uses ONLY turtlesim's own services: /clear, /turtle1/teleport_absolute and
/turtle1/set_pen (pen is lifted so no line is drawn while teleporting).
"""
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

START_X = 5.544445
START_Y = 5.544445
START_THETA = 0.0
DEFAULT_PEN = (179, 184, 255, 3)   # turtlesim default r, g, b, width


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        grp = ReentrantCallbackGroup()
        self.clear_cli = self.create_client(Empty, '/clear', callback_group=grp)
        self.tp_cli = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=grp)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen', callback_group=grp)
        self.create_service(Trigger, 'reset_turtle', self._reset_cb, callback_group=grp)
        self.get_logger().info('Reset service ready (service: reset_turtle)')

    def _call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise RuntimeError(f'timeout calling {client.srv_name}')
            time.sleep(0.01)
        return fut.result()

    def _set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = DEFAULT_PEN
        req.off = 1 if off else 0
        self._call(self.pen_cli, req)

    def _reset_cb(self, request, response):
        try:
            self._set_pen(off=True)                 # no line while teleporting
            self._call(self.clear_cli, Empty.Request())
            tp = TeleportAbsolute.Request()
            tp.x, tp.y, tp.theta = START_X, START_Y, START_THETA
            self._call(self.tp_cli, tp)
            self._set_pen(off=False)
            response.success = True
            response.message = 'Screen cleared, turtle back at start'
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
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
