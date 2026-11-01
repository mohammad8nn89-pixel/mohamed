"""Pure (ROS-free) go-to-pose controller.

Design pattern: STATE. The controller moves through the phases
ROTATE_TO_GOAL -> DRIVE -> ROTATE_TO_YAW -> DONE.
Keeping it free of ROS makes it easy to test and to reuse.
"""
import math
from enum import Enum, auto


def normalize_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Phase(Enum):
    ROTATE_TO_GOAL = auto()
    DRIVE = auto()
    ROTATE_TO_YAW = auto()
    DONE = auto()


class PoseController:
    K_LIN = 1.5
    K_ANG = 4.0
    K_HEADING = 6.0
    MIN_LIN = 0.2
    MIN_ANG = 0.2

    def __init__(self, target_x, target_y, target_yaw,
                 pos_tol=0.05, yaw_tol=0.02):
        self.tx, self.ty = float(target_x), float(target_y)
        self.tyaw = normalize_angle(float(target_yaw))
        self.pos_tol = pos_tol
        self.yaw_tol = yaw_tol
        self.phase = Phase.ROTATE_TO_GOAL

    @property
    def done(self) -> bool:
        return self.phase is Phase.DONE

    def distance(self, x, y) -> float:
        return math.hypot(self.tx - x, self.ty - y)

    def _rotate_cmd(self, err, max_ang):
        speed = max(self.K_ANG * abs(err), self.MIN_ANG)
        speed = min(speed, max_ang)
        return math.copysign(speed, err)

    def step(self, x, y, theta, max_lin, max_ang):
        """Return (linear, angular) velocity for the current pose."""
        dist = self.distance(x, y)
        heading_err = normalize_angle(math.atan2(self.ty - y, self.tx - x) - theta)

        if self.phase is Phase.ROTATE_TO_GOAL:
            if dist < self.pos_tol:
                self.phase = Phase.ROTATE_TO_YAW
            elif abs(heading_err) < 0.02:
                self.phase = Phase.DRIVE
            else:
                return 0.0, self._rotate_cmd(heading_err, max_ang)

        if self.phase is Phase.DRIVE:
            if dist < self.pos_tol:
                self.phase = Phase.ROTATE_TO_YAW
            else:
                lin = min(max(self.K_LIN * dist, self.MIN_LIN), max_lin)
                if abs(heading_err) > 0.5:      # badly misaligned: turn first
                    lin = 0.0
                ang = clip(self.K_HEADING * heading_err, max_ang)
                return lin, ang

        if self.phase is Phase.ROTATE_TO_YAW:
            yaw_err = normalize_angle(self.tyaw - theta)
            if abs(yaw_err) < self.yaw_tol:
                self.phase = Phase.DONE
            else:
                return 0.0, self._rotate_cmd(yaw_err, max_ang)

        return 0.0, 0.0
