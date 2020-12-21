import transformations as tf
import numpy as np
from numpy import ndarray

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage
from tf2_py import LookupException
from tf2_ros.buffer import Buffer


def pose_to_matrix(pose: Pose) -> ndarray:
    rot = pose.orientation
    trans = pose.position

    rot_matrix = tf.quaternion_matrix((rot.w, rot.x, rot.y, rot.z))
    trans_matrix = tf.translation_matrix((trans.x, trans.y, trans.z))

    return tf.concatenate_matrices(trans_matrix, rot_matrix)


def matrix_to_pose(matrix):
    pose = Pose()
    trans = tf.translation_from_matrix(matrix)
    quat = tf.quaternion_from_matrix(matrix)

    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.w = quat[0]
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]

    return pose


def midpoint(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return interpolate(a, b, phase=0.5)


def interpolate(start: np.ndarray, stop: np.ndarray, phase: float) -> np.ndarray:
    start_trans = tf.translation_from_matrix(start)
    stop_trans = tf.translation_from_matrix(stop)
    shift_trans = phase * (stop_trans - start_trans)
    inter_trans = start_trans + shift_trans
    trans_matrix = tf.translation_matrix(inter_trans)

    start_rot = np.array(tf.euler_from_matrix(start, axes='sxyz'))
    stop_rot = np.array(tf.euler_from_matrix(stop, axes='sxyz'))
    shift_rot = phase * (stop_rot - start_rot)
    inter_rot = start_rot + shift_rot
    rot_matrix = tf.euler_matrix(*inter_rot, axes='sxyz')

    result = tf.concatenate_matrices(trans_matrix, rot_matrix)
    return result


def get_transform_lookup(tf_buffer: Buffer):
    def transform_lookup(target: str, source: str) -> ndarray:
        MAX_TRIES = 10
        attempt = 1
        while attempt < MAX_TRIES:
            try:
                transform = tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.5))
            except LookupException:
                attempt += 1
                if attempt >= MAX_TRIES:
                    raise
                print("Couldn't transform, retrying...")
            else:
                break
        transform = transform.transform
        rot = transform.rotation
        trans = transform.translation

        rot_matrix = tf.quaternion_matrix((rot.w, rot.x, rot.y, rot.z))
        trans_matrix = tf.translation_matrix((trans.x, trans.y, trans.z))

        # WARNING! Correct order of multiplication! trans * rot
        return tf.concatenate_matrices(trans_matrix, rot_matrix)
    return transform_lookup


def get_transform_publisher(broadcaster, clock):
    def publish_transform(src: str, dst: str, pose: np.ndarray):
        trans = tf.translation_from_matrix(pose)
        rot = tf.quaternion_from_matrix(pose)

        transform = TransformStamped()
        transform.header.stamp = header_stamp(clock.now())
        transform.header.frame_id = src
        transform.child_frame_id = dst
        transform.transform.translation.x = trans[0]
        transform.transform.translation.y = trans[1]
        transform.transform.translation.z = trans[2]
        transform.transform.rotation.w = rot[0]
        transform.transform.rotation.x = rot[1]
        transform.transform.rotation.y = rot[2]
        transform.transform.rotation.z = rot[3]

        msg = TFMessage(transforms=[transform])
        broadcaster.publish(msg)
    return publish_transform


def header_stamp(time):
    stamp = Time(
        sec=time.nanoseconds // 10**9,
        nanosec=time.nanoseconds % 10**9
    )
    return stamp
