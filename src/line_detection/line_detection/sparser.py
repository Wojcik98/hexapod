import os
import platform
import time
from typing import List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import transformations as tf
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from hex_control.transformations_utils import get_transform_lookup, \
    get_transform_publisher, interpolate, matrix_to_pose, pose_to_matrix

Matrix = np.ndarray

PACKAGE = 'line_detection'


def dist(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5


def matrix_dist(a: Matrix, b: Matrix):
    return dist(
        tf.translation_from_matrix(a),
        tf.translation_from_matrix(b)
    )


class Sparser(Node):
    def __init__(self):
        super().__init__('sparser')
        self.stride = 0.1

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.transform = get_transform_lookup(self._tf_buffer)
        self.tf_broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.publish_transform = get_transform_publisher(self.tf_broadcaster,
                                                         self.get_clock())

        # self.publish_transform('odom', 'base_link', tf.translation_matrix((0, 0, 0)))
        self.pub = self.create_publisher(Path, 'sparse_path', 10)
        # self.publish_transform('odom', 'front_point', start_pose)
        self.curr_path = []
        self.publish_path(self.curr_path)

        self.subscription = self.create_subscription(
            Path,
            'dense_path',
            self.new_trajectory,
            10
        )

    def new_trajectory(self, msg: Path):
        if len(self.curr_path) == 0:
            self.curr_path.append(self.transform('odom', 'front_point'))
        # self.publish_transform('odom', 'base_link', tf.translation_matrix((0, 0, 0)))
        projection_to_odom = self.transform('odom', 'base_link')
        projection_to_odom[2, 3] = 0.0
        dense_path = [
            tf.concatenate_matrices(
                pose_to_matrix(pose_stamped.pose),
                projection_to_odom
            )
            for pose_stamped in msg.poses
        ]
        # dense_path = [
        #     pose_to_matrix(pose_stamped.pose)
        #     for pose_stamped in msg.poses
        # ]

        first_new = dense_path[0]
        last_idx, last_point = self.get_last_closest(first_new, self.curr_path)
        inter_path = self.interpolate_path(last_point, first_new)
        sparse_path = self.make_sparse(inter_path[-1], dense_path)

        self.curr_path = self.curr_path[:last_idx] + inter_path + sparse_path
        front_point = self.transform('odom', 'front_point')
        front_point_idx, _ = self.get_last_closest(front_point, self.curr_path)
        self.publish_path(self.curr_path[front_point_idx + 1:])

    def publish_path(self, path: List[Matrix]):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9

        poses = [matrix_to_pose(point) for point in path]
        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            msg.poses.append(pose_stamped)

        self.pub.publish(msg)
        print('Sparse path published!')

    def get_last_closest(self, new_point: Matrix, path: List[Matrix]) -> Tuple[int, Matrix]:
        min_dist = float('inf')
        min_point = None
        min_idx = None

        for i, point in enumerate(path):
            d = matrix_dist(point, new_point)

            if d < min_dist:
                min_dist = d
                min_point = point.copy()
                min_idx = i

        return min_idx, min_point

    def interpolate_path(self, start_point: Matrix, stop_point: Matrix) -> List[Matrix]:
        interpolated_points = [start_point]
        prev = start_point
        d = matrix_dist(prev, stop_point)

        while d > self.stride:
            ratio = self.stride / d
            inter = interpolate(prev, stop_point, ratio)
            interpolated_points.append(inter)
            prev = inter
            d = matrix_dist(prev, stop_point)

        return interpolated_points

    def make_sparse(self, start_point: Matrix, dense_path: List[Matrix]) -> List[Matrix]:
        sparse_path = []
        prev_point = start_point
        remaining_path = dense_path

        while len(remaining_path) > 0:
            idx, point = self.get_farthest_in_range(prev_point, remaining_path)
            sparse_path.append(point)
            prev_point = point

            if idx + 1 == len(remaining_path):
                remaining_path = []
            else:
                remaining_path = remaining_path[idx + 1:]

        return sparse_path

    def get_farthest_in_range(self, start_point: Matrix, path: List[Matrix]) -> Tuple[int, Matrix]:
        farthest = 0

        for i, point in enumerate(path):
            d = dist(
                tf.translation_from_matrix(point),
                tf.translation_from_matrix(start_point)
            )

            if d > self.stride:
                break
            else:
                farthest = i

        return farthest, path[farthest].copy()


def main(args=None):
    print('Initializing ROS...')
    rclpy.init(args=args)

    print('Create sparser...')
    sparser = Sparser()

    print('Sparser ready!')
    rclpy.spin(sparser)
    print('Finished!')

    sparser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
