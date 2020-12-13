from dataclasses import dataclass
import transformations as tf


@dataclass
class TrajectoryPoint:
    left_moving = True
    phase = 0.0
    timestamp = 0.0
    left_pose = tf.translation_matrix((0, 0, 0))
    right_pose = tf.translation_matrix((0, 0, 0))
    prev_left_stand = tf.translation_matrix((0, 0, 0))
    prev_right_stand = tf.translation_matrix((0, 0, 0))
    next_left_stand = tf.translation_matrix((0, 0, 0))
    next_right_stand = tf.translation_matrix((0, 0, 0))
    base_link_pose = tf.translation_matrix((0, 0, 0))
