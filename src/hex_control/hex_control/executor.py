from time import sleep
import transformations as tf

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from hex_control.path_proxy import PathProxy
from hex_control.tripod_inverse_kinematics import TripodInverseKinematics
from hex_control.trajectory_point import TrajectoryPoint
from hex_control.trajectory_generator import TrajectoryGenerator
from hex_control.trajectory_generator import SERVO_FREQ
from hex_control.trajectory_encoder import TrajectoryEncoder
from hex_control.transformations_utils import get_transform_publisher, \
    header_stamp, pose_to_matrix

UPDATE_PERIOD = 1.0
START_DELAY = UPDATE_PERIOD / 2


class Executor(Node):
    SET_SERVOS_NOW = 42
    SET_SERVOS_LOOP = 43

    def __init__(self):
        super().__init__('executor')

        """Update frequency
        Single targets: {baudrate} / ({uart frame length} * {cmd length} * {number of joint})
        = 115200 / (9 * 4 * 18) ~= 177Hz <- autodetect-baudrate on maestro
        = 200000 / (9 * 4 * 1x8) ~= 300Hz <- fixed-baudrate on maestro
        Multiple targets: {baudrate} / ({uart frame length} * ({header length} + {mini-cmd length} * {number of joints}))
        = 115200 / (9 * (3 + 2 * 18)) ~= 320Hz 
        = 200000 / (9 * (3 + 2 * 18)) ~= 560Hz"""

        self.subscription = self.create_subscription(
            Path,
            'sparse_path',
            self.new_trajectory,
            10
        )
        self.spi_bridge = self.create_publisher(
            UInt8MultiArray, 'stm32_cmd', 10
        )

        self.steps_trajectory = []
        self.init_test_path()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.broadcaster = self.create_publisher(TFMessage, '/tf', 10)
        self.publish_transform = get_transform_publisher(self.broadcaster, self.get_clock())

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.base_link_start_height = tf.translation_matrix((0.0, 0.0, 0.15))

        self.path_proxy = PathProxy(self.steps_trajectory, self.get_logger())
        self.trajectory_generator = TrajectoryGenerator(self.path_proxy)
        self.trajectory_encoder = TrajectoryEncoder()
        self.front_point = Pose()
        self.rear_point = Pose()
        self.init_points()

        self.inv_kin_calc = TripodInverseKinematics(self._tf_buffer, self.get_clock())

        self.prev_time = self.get_clock().now()

        first_step = TrajectoryPoint()
        first_step.timestamp = self.prev_time.nanoseconds / 10**9
        first_step.left_moving = True
        first_step.left_pose = tf.translation_matrix((0, 0, 0))
        first_step.right_pose = tf.translation_matrix((0, 0, 0))
        self.prev_trajectory = [first_step]

        self.first_time = False
        self.start_timer = self.create_timer(START_DELAY, self.timer_callback)
        self.timer = None
        self.tf_viz_tim = None

    def new_trajectory(self, msg: Path):
        new_path = [
            pose_to_matrix(pose_stamped.pose)
            for pose_stamped in msg.poses
        ]
        self.steps_trajectory.clear()
        self.steps_trajectory.extend(new_path)

    def timer_callback(self):
        if self.start_timer is not None and not self.first_time:
            print('first')
            self.first_time = True
        elif self.first_time:
            print('second')
            self.start_timer.cancel()
            self.start_timer = None
            self.timer = self.create_timer(UPDATE_PERIOD, self.timer_callback)
            self.tf_viz_tim = self.create_timer(0.1, self.tf_viz_callback)

        start = self.get_clock().now()
        # diff = start.nanoseconds - self.prev_time.nanoseconds
        # point_i = int(SERVO_FREQ * diff / (10 ** 9))
        # print(diff/10**9, point_i)
        # if point_i >= len(self.prev_trajectory):
        #     point_i = len(self.prev_trajectory) - 1
        # print(point_i)
        current_point = self.prev_trajectory[-1]

        new_trajectory = self.trajectory_generator.generate_trajectory(current_point)

        inversed_trajectory = [
            self.inv_kin_calc.calc_point(point)
            for point in new_trajectory
        ]
        data = self.trajectory_encoder.encode_trajectory(inversed_trajectory)

        cmd = self.SET_SERVOS_NOW if self.first_time else self.SET_SERVOS_LOOP
        data = bytearray([cmd]) + data
        spi_msg = UInt8MultiArray(data=data)
        self.spi_bridge.publish(spi_msg)

        self.prev_trajectory.extend(new_trajectory)

        front_point = self.path_proxy.first_unused()
        triangle = self.inv_kin_calc.triangle(front_point, left_side=False)
        self.publish_transform('odom', 'front_point', front_point)
        self.publish_transform('odom', 'tri_a', triangle[0])
        self.publish_transform('odom', 'tri_b', triangle[1])
        self.publish_transform('odom', 'tri_c', triangle[2])

        elapsed = self.get_clock().now() - start
        print(f'Elapsed {elapsed.nanoseconds / 10**9:.3f}s')

        if self.start_timer is None:
            self.first_time = False

    def init_points(self):
        if self._tf_buffer.can_transform('odom', 'front_point', Time()):
            transform = self._tf_buffer.lookup_transform(
                'odom', 'front_point', Time())
            self.front_point.position.x = transform.transform.translation.x
            self.front_point.position.y = transform.transform.translation.y
            self.front_point.position.z = transform.transform.translation.z
        else:
            # TODO remove, it should be done in Sparser
            self.publish_transform(
                'odom', 'front_point', tf.translation_matrix((0, 0, 0)))

        if self._tf_buffer.can_transform('odom', 'rear_point', Time()):
            transform = self._tf_buffer.lookup_transform(
                'odom', 'rear_point', Time())
            self.rear_point.position.x = transform.transform.translation.x
            self.rear_point.position.y = transform.transform.translation.y
            self.rear_point.position.z = transform.transform.translation.z
        else:
            self.publish_transform(
                'odom', 'rear_point', tf.translation_matrix((0, 0, 0)))

        while self.pub.get_subscription_count() < 1:
            pass
        sleep(2)
        self.publish_transform('odom', 'base_link', self.base_link_start_height)

    def init_test_path(self):
        self.steps_trajectory = [
            tf.translation_matrix((0, 0, 0)),
        ]

    def tf_viz_callback(self):
        now = self.get_clock().now()
        point_i = int(SERVO_FREQ * (now.nanoseconds - self.prev_time.nanoseconds) / (10**9))  # TODO
        if point_i >= len(self.prev_trajectory):
            point_i = len(self.prev_trajectory) - 1
        point = self.prev_trajectory[point_i]
        angles = self.inv_kin_calc.calc_point(point)
        # print(angles)

        names, positions = [], []
        for name, position in angles.items():
            names.append(name)
            positions.append(position)

        msg = JointState()
        msg.header.stamp = header_stamp(self.get_clock().now())
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)

        # base = tf.concatenate_matrices(point.base_link_pose, self.base_link_height)
        base = point.base_link_pose
        self.publish_transform('odom', 'base_link', base)
        projection = base.copy()
        projection[2, 3] = 0.0
        self.publish_transform('odom', 'base_projection', projection)
        self.publish_transform('odom', 'right_tri', point.right_pose)
        self.publish_transform('odom', 'left_tri', point.left_pose)


def main(args=None):
    rclpy.init(args=args)

    executor = Executor()

    print('Executor ready!')

    rclpy.spin(executor)

    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
