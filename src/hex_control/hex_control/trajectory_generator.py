import transformations as tf
from math import cos, pi, sin
from typing import List

from hex_control.path_proxy import PathProxy
from hex_control.trajectory_point import TrajectoryPoint
from hex_control.transformations_utils import midpoint, interpolate

SERVO_FREQ = 250
SENT_TRAJECTORY_TIME = 1
TRAJECTORY_TIME = 1.
GAIT_PERIOD = 1.0


class TrajectoryGenerator:
    def __init__(self, path_proxy: PathProxy):
        self.path_proxy = path_proxy
        self.leg_lift = 0.1
        self.base_height = 0.15

    def generate_trajectory(self, start_step: TrajectoryPoint) -> List[TrajectoryPoint]:
        trajectory = []
        point = start_step

        for i in range(int(TRAJECTORY_TIME * SERVO_FREQ)):
            point = self.make_next_step(point)
            trajectory.append(point)

        return trajectory

    def make_next_step(self, prev_step: TrajectoryPoint) -> TrajectoryPoint:
        point = TrajectoryPoint()
        point.timestamp = prev_step.timestamp + 1 / SERVO_FREQ
        phase = prev_step.phase + 1 / (SERVO_FREQ * GAIT_PERIOD)

        # z_lift = self.leg_lift * sin(phase * pi)
        z_lift = self.leg_lift * (-cos(phase * pi * 2) + 1) / 2
        current_lift = tf.translation_matrix((0.0, 0.0, z_lift))

        if phase > 1.0:
            phase = 0
            point.left_pose = prev_step.next_left_stand
            point.right_pose = prev_step.next_right_stand

            next_stand = self.path_proxy.get_point(point.timestamp)

            if prev_step.left_moving:
                point.prev_left_stand = prev_step.next_left_stand
                point.next_left_stand = prev_step.next_left_stand
                point.prev_right_stand = prev_step.next_right_stand
                point.next_right_stand = next_stand
            else:
                point.prev_right_stand = prev_step.next_right_stand
                point.next_right_stand = prev_step.next_right_stand
                point.prev_left_stand = prev_step.next_left_stand
                point.next_left_stand = next_stand
            point.left_moving = not prev_step.left_moving
        else:
            if prev_step.left_moving:
                left_shift = interpolate(prev_step.prev_left_stand,
                                         prev_step.next_left_stand,
                                         phase)
                point.left_pose = tf.concatenate_matrices(
                    # prev_step.prev_left_stand,
                    left_shift,
                    current_lift)
                point.right_pose = prev_step.right_pose
            else:
                right_shift = interpolate(prev_step.prev_right_stand,
                                          prev_step.next_right_stand,
                                          phase)
                point.right_pose = tf.concatenate_matrices(
                    # prev_step.prev_right_stand,
                    right_shift,
                    current_lift)
                point.left_pose = prev_step.left_pose

            point.next_left_stand = prev_step.next_left_stand
            point.next_right_stand = prev_step.next_right_stand
            point.prev_left_stand = prev_step.prev_left_stand
            point.prev_right_stand = prev_step.prev_right_stand
            point.left_moving = prev_step.left_moving

        point.phase = phase
        point.base_link_pose = midpoint(point.left_pose, point.right_pose)
        point.base_link_pose[2, 3] = self.base_height

        return point
