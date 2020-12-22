import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from math import pi

from hex_control.executor import Executor
from hex_control.trajectory_encoder import TrajectoryEncoder
from std_msgs.msg import UInt8MultiArray

SPI_SPEED = 1000000


class JointStatePub(Node):
    def __init__(self):
        super().__init__('joint_state_pub')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.spi_bridge = self.create_publisher(
            UInt8MultiArray, 'stm32_cmd', 10
        )

        self.encoder = TrajectoryEncoder()
        self.spi = None

        self.timer_period = 1.0
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        names = []
        positions = []

        desired = {'coxa_joint_l1': 1.1676793609514362,
                   'femur_joint_l1': 0.37447060256083464,
                   'tibia_joint_l1': -1.9052273890701883,
                   'coxa_joint_r2': -1.5707963267948966,
                   'femur_joint_r2': 0.3393852618985336,
                   'tibia_joint_r2': -1.7052129720975133,
                   'coxa_joint_l3': 1.9739132926383571,
                   'femur_joint_l3': 0.37447060256083464,
                   'tibia_joint_l3': -1.9052273890701883,
                   'coxa_joint_r1': -1.1676793609514362,
                   'femur_joint_r1': 1.7272784703057642,
                   'tibia_joint_r1': -2.5150412097293935,
                   'coxa_joint_l2': 1.5707963267948966,
                   'femur_joint_l2': 1.4429503104838526,
                   'tibia_joint_l2': -2.2342414561008335,
                   'coxa_joint_r3': -1.9739132926383571,
                   'femur_joint_r3': 1.7272784703057642,
                   'tibia_joint_r3': -2.5150412097293935}

        segments = ['coxa', 'femur', 'tibia']
        legs = ['l1', 'l2', 'l3', 'r1', 'r2', 'r3']

        if desired is None:
            for seg in segments:
                for leg in legs:
                    names.append(f'{seg}_joint_{leg}')
                    if seg[0] == 'c':
                        if leg[0] == 'l':
                            positions.append(pi / 2)
                        else:
                            positions.append(-pi / 2)
                    elif seg[0] == 'f':
                        positions.append(pi / 4)
                    else:
                        positions.append(-2 * pi / 3)
        else:
            for name, position in desired.items():
                names.append(name)
                positions.append(position)

        angles = {key: val for key, val in zip(names, positions)}
        data = self.encoder.encode_step(angles)
        data = data * 250

        cmd = Executor.SET_SERVOS_NOW
        data = bytearray([cmd]) + data
        spi_msg = UInt8MultiArray(data=data)
        self.spi_bridge.publish(spi_msg)

        msg = JointState()
        msg.header.stamp = Time()
        msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9
        msg.name = names
        msg.position = positions

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_pub = JointStatePub()

    print('Ready!')
    rclpy.spin(joint_state_pub)

    joint_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
