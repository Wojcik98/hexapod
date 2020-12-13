import rclpy
# import spidev
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from math import pi

from hex_control.trajectory_encoder import TrajectoryEncoder

SPI_SPEED = 1000000


class JointStatePub(Node):
    def __init__(self):
        super().__init__('joint_state_pub')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.encoder = TrajectoryEncoder()
        # try:
        #     self.spi = spidev.SpiDev()
        #     self.spi.open(0, 0)
        #     self.spi.max_speed_hz = SPI_SPEED
        # except FileNotFoundError:
        #     pass
        self.spi = None

        self.i = 0

        self.timer_period = 1.0
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        names = []
        positions = []

        segments = ['coxa', 'femur', 'tibia']
        legs = ['l1', 'l2', 'l3', 'r1', 'r2', 'r3']

        for seg in segments:
            for leg in legs:
                names.append(f'{seg}_joint_{leg}')
                if seg[0] == 'c':
                    if leg[0] == 'l':
                        positions.append(pi / 2)
                    else:
                        positions.append(-pi / 2)
                elif seg[0] == 'f':
                    positions.append(pi / 8)
                else:
                    if self.i % 2:
                        positions.append(-pi / 8)
                    else:
                        positions.append(pi / 8)

        angles = {key: val for key, val in zip(names, positions)}
        data = self.encoder.encode_step(angles)
        if self.spi:
            data = bytearray([42]) + data
            self.spi.xfer3(data)
            print(f"Sent data, length {len(data)}, content: {data}")

        msg = JointState()
        msg.header.stamp = Time()
        msg.header.stamp.sec = int(self.get_clock().now().nanoseconds / 10**9)
        msg.header.stamp.nanosec = self.get_clock().now().nanoseconds % 10**9
        msg.name = names
        msg.position = positions

        self.pub.publish(msg)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    joint_state_pub = JointStatePub()

    print('Ready!')
    rclpy.spin(joint_state_pub)

    joint_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
