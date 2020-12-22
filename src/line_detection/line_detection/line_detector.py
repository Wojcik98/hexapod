import os
import platform
import time
from typing import List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import transformations as tf
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from math import pi
from nav_msgs.msg import Path
from rclpy.node import Node
from scipy.ndimage import uniform_filter1d

from hex_control.transformations_utils import matrix_to_pose

Matrix = np.ndarray

PACKAGE = 'line_detection'
UPDATE_INTERVAL = 0.5


def dist(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        pkg_path = get_package_share_directory(PACKAGE)
        transform_file_path = os.path.join(pkg_path, 'image_transform.yaml')
        with open(transform_file_path, 'r') as f:
            content = f.read()
            self.M = np.array(yaml.load(content))
            print(self.M)

        # self._tf_buffer = Buffer()
        # self._tf_listener = TransformListener(self._tf_buffer, self)
        # self.transform = get_transform_lookup(self._tf_buffer)
        self.pub = self.create_publisher(Path, 'dense_path', 10)
        self.rpi = platform.machine() == 'aarch64'

        if self.rpi:
            self.camera = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        else:
            self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.create_timer(UPDATE_INTERVAL, self.take_photo)

    def take_photo(self):
        # if self.rpi:
        #     _, image = self.camera.read()
        # else:
        #     image = cv2.imread('/home/michal/orig.jpg')
        _, image = self.camera.read()
        # plt.imsave('/home/ubuntu/orig.jpg', image)
        start = time.time()
        start_start = start

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        stop = time.time()
        print(f'To gray: {1000 * (stop - start):.3f}ms')
        start = stop

        _, thres = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        stop = time.time()
        print(f'Thresholded: {1000 * (stop - start):.3f}ms')
        start = stop

        cont, hier = cv2.findContours(thres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        raw_path = max(cont, key=cv2.contourArea).astype("float64")
        stop = time.time()
        print(f"Contour: {1000 * (stop - start):.3f}ms")
        start = stop

        print(f'Elapsed {1000 * (stop - start_start):.3f}ms')

        path_shifted = self.shift_path(raw_path)
        path = cv2.perspectiveTransform(path_shifted, self.M)
        path = path.reshape((len(path), 2))
        subpath = self.get_subpath(path)

        # if going in another direction, abort
        # TODO make it  smarter
        if subpath[0][0] < 0.01:
            return

        prev_point = (0.0, 0.0)
        oriented = self.add_orientation(prev_point, subpath)
        print(tf.euler_from_matrix(oriented[0])[2] * 180 / pi)
        print(tf.translation_from_matrix(oriented[0]))

        self.publish_path(oriented)

    def publish_path(self, path: List[Matrix]):
        msg = Path()
        msg.header.frame_id = 'base_link'
        msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9

        poses = [matrix_to_pose(point) for point in path]
        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            msg.poses.append(pose_stamped)

        self.pub.publish(msg)
        print('Dense path published!')

    def shift_path(self, path):
        start_shape = path.shape
        path = path.reshape((path.shape[0], 2))
        top = path[:, 1] == np.max(path[:, 1], axis=0)
        line_start = np.nonzero(path[top, 0] == np.min(path[top, 0], axis=0))[0][0]
        line_dir = 1 if path[line_start + 1, 1] > path[line_start, 1] else -1
        line_shifted = np.roll(path, -line_start)
        if line_dir < 0:
            line_shifted = np.roll(np.flip(line_shifted), 1)

        return line_shifted.reshape(start_shape)

    def get_subpath(self, path):
        LEN_THRESHOLD = 0.2
        total_len = 0.0
        i = 1
        while total_len < LEN_THRESHOLD:
            total_len += dist(path[i - 1], path[i])
            i += 1
            if i >= len(path):
                break

        line_end = i
        return path[:line_end]

    def add_orientation(self, prev_point:Tuple[float, float],
                        path: List[Tuple[float, float]]) -> List[Matrix]:
        new_path = []
        for point in path:
            x_prev, y_prev = prev_point
            x_curr, y_curr = point
            x, y, = x_curr - x_prev, y_curr - y_prev
            rot = np.arctan2(y, x)

            oriented_point = tf.concatenate_matrices(
                tf.translation_matrix((x_curr, y_curr, 0)),
                tf.rotation_matrix(rot, (0, 0, 1))
            )
            new_path.append(oriented_point)

            prev_point = point

        return new_path


def main(args=None):
    print('Initializing ROS...')
    rclpy.init(args=args)

    print('Create interface...')
    detector = LineDetector()

    print('Line detector ready!')
    print('Start analyzing...')
    # detector.send_msg()
    print('Finished!')
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
