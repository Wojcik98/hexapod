import os
import platform
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

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

        # # blurd = filters.gaussian(gray, sigma=30)
        # blurd = gaussian_filter(gray, sigma=3)
        # stop = time.time()
        # print(f'Blurred: {1000 * (stop - start):.3f}ms')
        # start = stop

        _, thres = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        stop = time.time()
        print(f'Thresholded: {1000 * (stop - start):.3f}ms')
        start = stop

        cont, hier = cv2.findContours(thres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        raw_path = max(cont, key=cv2.contourArea).astype("float64")
        # raw_path = raw_path.reshape((raw_path.shape[0], 2))
        # output = np.zeros((480, 640), dtype=np.uint8)
        # cv2.drawContours(output, [raw_path], -1, 1, thickness=cv2.FILLED)
        stop = time.time()
        print(f"Contour: {1000 * (stop - start):.3f}ms")
        start = stop

        print(f'Elapsed {1000 * (stop - start_start):.3f}ms')

        path_shifted = self.shift_path(raw_path)
        path = cv2.perspectiveTransform(path_shifted, self.M)
        path = path.reshape((len(path), 2))
        subpath = self.get_subpath(path)

        # SENDING MESSAGE

        msg = Path()
        # msg.header.frame_id = 'base_projection'
        msg.header.frame_id = 'base_link'
        msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9

        for point in subpath:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0  # TODO height from base_to_odom or represent it in odom reference
            msg.poses.append(pose_stamped)

        self.pub.publish(msg)
        print('Sent')

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
