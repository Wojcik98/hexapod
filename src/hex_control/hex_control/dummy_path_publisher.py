from math import pi

import rclpy
import transformations as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from hex_control.transformations_utils import get_transform_lookup, matrix_to_pose


class DummyPathPublisher(Node):
    def __init__(self):
        super().__init__('dummy_path_publisher')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.transform = get_transform_lookup(self._tf_buffer)

        # self.pub = self.create_publisher(Path, 'sparse_path', 10)
        self.pub = self.create_publisher(Path, 'dense_path', 10)

        self.msg_sent = False
        self.create_timer(1, self.send_msg)

    def send_msg(self):
        if self.msg_sent:
            return

        print('Sending msg')
        self.msg_sent = True
        front_point = self.transform('odom', 'front_point')
        inc = tf.concatenate_matrices(
            tf.rotation_matrix(-5 * pi / 180, (0, 0, 1)),
            tf.translation_matrix((0.03, 0.01, 0)),
        )
        path = [tf.concatenate_matrices(front_point, inc)]
        for i in range(179):
            path.append(tf.concatenate_matrices(path[-1], inc))

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
        print('Sent')


def main(args=None):
    rclpy.init(args=args)

    executor = DummyPathPublisher()

    print('Dummy path publisher ready!')
    rclpy.spin(executor)

    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
