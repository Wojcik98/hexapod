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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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
        # self.raw_img_pub = self.create_publisher(Image, 'camera_image', 10)
        self.contour_pub = self.create_publisher(Image, 'line_contour', 10)
        self.bridge = CvBridge()

        self.rpi = platform.machine() == 'aarch64'
        if self.rpi:
            self.camera = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        else:
            self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.create_timer(UPDATE_INTERVAL, self.take_photo)

    def take_photo(self):
        # if self.rpi:
        #     _, image = self.camera.read()
        # else:
        #     image = cv2.imread('/home/michal/orig.jpg')
        for _ in range(2):  # camera has strange buffer and we need to bypass it
            _, image = self.camera.read()
        _, image = self.camera.read()
        # imgmsg = self.bridge.cv2_to_imgmsg(image)
        # imgmsg.header.frame_id = 'odom'
        # imgmsg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        # imgmsg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9
        # self.raw_img_pub.publish(imgmsg)
        # plt.imsave('/home/ubuntu/orig.jpg', image)
        start = time.time()
        start_start = start

        sigma = 2
        blur = cv2.GaussianBlur(image, (0, 0), sigma)
        blur = cv2.GaussianBlur(blur, (0, 0), sigma)
        blur = cv2.GaussianBlur(blur, (0, 0), sigma)
        stop = time.time()
        print(f"Blurring: {1000 * (stop - start):.3f}ms")
        start = stop

        imghsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)
        white_lower_hsv = np.array([0, 0, 150])
        white_upper_hsv = np.array([179, 80, 255])
        masked = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
        stop = time.time()
        print(f"Masking color: {1000 * (stop - start):.3f}ms")
        start = stop

        cont, hier = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(cont) == 0:
            print("No contours detected!")
            return
        # self.get_logger().info(f"Cont: {cont}, hier: {hier}")
        raw_path = max(cont, key=cv2.contourArea)
        print(raw_path)
        stop = time.time()
        print(f"Contour: {1000 * (stop - start):.3f}ms")
        start = stop

        print(f'Elapsed {1000 * (stop - start_start):.3f}ms')

        path_shifted = self.shift_path(raw_path.astype("float64"))
        # contour_img = np.zeros((480, 640), dtype=np.uint8)
        contour_img = image.copy()
        cv2.drawContours(contour_img, [raw_path], -1, 255, thickness=3)  # TODO draw on original?
        cont_msg = self.bridge.cv2_to_imgmsg(contour_img)
        cont_msg.header.frame_id = 'odom'
        cont_msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        cont_msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9
        self.contour_pub.publish(cont_msg)

        path = cv2.perspectiveTransform(path_shifted, self.M)
        path = path.reshape((len(path), 2))
        # TODO take only longest positive y derivative?
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
        """Shifts path so it starts at its top left point"""
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
