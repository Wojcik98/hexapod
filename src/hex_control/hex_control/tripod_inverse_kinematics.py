import json
import os
import transformations as tf
from dataclasses import dataclass
from math import acos, atan2, pi, sqrt
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
from numpy import ndarray
from rclpy.clock import Clock
from tf2_ros.buffer import Buffer

from hex_control.trajectory_point import TrajectoryPoint
from hex_control.transformations_utils import get_transform_lookup


@dataclass
class TransformCache:
    timestamp: float
    transform: ndarray


class TripodInverseKinematics:
    CACHE_TIMEOUT = 0.1

    def __init__(self, buffer: Buffer, clock: Clock):
        self.clock = clock
        self._tf_buffer = buffer
        self.transform_lookup = get_transform_lookup(self._tf_buffer)
        self.transform_cache: Dict[str, TransformCache] = {}
        self.leg_desc = self.get_leg_desc()
        self.base_to_hip = {}

    def init_transforms(self):
        legs = ['l1', 'l2', 'l3', 'r1', 'r2', 'r3']
        self.base_to_hip = {
            leg: self.transform_lookup('base_link', f'leg_center_{leg}')
            for leg in legs
        }

    def calc_point(self, point: TrajectoryPoint) -> Dict[str, float]:
        angles = {}

        angles.update(self.calc_legs(point.left_pose, point.base_link_pose, left_side=True))
        angles.update(self.calc_legs(point.right_pose, point.base_link_pose, left_side=False))

        return angles

    def calc_legs(self,
                  pose: ndarray,
                  base_pose: ndarray,
                  left_side: bool) -> Dict[str, float]:
        if len(self.base_to_hip) == 0:
            self.init_transforms()
        names = []
        positions = []

        legs_positions = self.triangle(pose, left_side)
        legs = ['l1', 'r2', 'l3'] if left_side else ['r1', 'l2', 'r3']

        for leg, position in zip(legs, legs_positions):
            base_to_hip = self.base_to_hip[leg]
            relative = tf.concatenate_matrices(
                tf.inverse_matrix(base_to_hip),
                tf.inverse_matrix(base_pose),
                position)
            point = tf.translation_from_matrix(relative)
            a, b, g = self.inverse_kinematics(*point)
            positions += [a, b, g]
            names += [f'coxa_joint_{leg}', f'femur_joint_{leg}', f'tibia_joint_{leg}']

        return {name: position for name, position in zip(names, positions)}

    def triangle(self, center: ndarray, left_side: bool) -> Tuple[ndarray, ndarray, ndarray]:
        mult = 1 if left_side else -1
        x_shift = 0.13
        side_y_shift = mult * 0.17
        mid_y_shift = mult * -0.2
        z = 0.0
        front_point = tf.concatenate_matrices(center, tf.translation_matrix((x_shift, side_y_shift, z)))
        mid_point = tf.concatenate_matrices(center, tf.translation_matrix((0.0, mid_y_shift, z)))
        rear_point = tf.concatenate_matrices(center, tf.translation_matrix((-x_shift, side_y_shift, z)))

        return front_point, mid_point, rear_point

    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        alpha = atan2(y, x)     # TODO use d
        w = sqrt(x**2 + y**2) - self.leg_desc[1]['a']
        z = z - self.leg_desc[1]['d']
        a = self.leg_desc[2]['a']
        b = self.leg_desc[3]['a']
        r = sqrt(w**2 + z**2)
        delta = acos((a**2 + b**2 - r**2) / (2 * a * b))
        gamma = pi - delta

        b1 = atan2(w, -z)
        b2 = acos((a**2 + r**2 - b**2) / (2 * a * r))
        beta = b1 + b2

        return float(alpha), float(beta - pi/2), float(-gamma)

    def get_leg_desc(self) -> List[Dict[str, float]]:
        params_path = os.path.join(
            get_package_share_directory('hex_description'),
            'models',
            'hex_description.json'
        )
        with open(params_path, 'r') as file:
            legs_params = json.load(file)
        return legs_params['general']

    def cached_transform_lookup(self, src, dst):
        id_ = f'{src} -> {dst}'
        now = self.clock.now().nanoseconds / 10**9

        if id_ not in self.transform_cache or \
           (now - self.transform_cache[id_].timestamp) > self.CACHE_TIMEOUT:
            transform = self.transform_lookup(src, dst)
            self.transform_cache[id_] = TransformCache(now, transform)
        return self.transform_cache[id_].transform
