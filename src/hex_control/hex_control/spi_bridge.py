import spidev

import rclpy
from std_msgs.msg import UInt8MultiArray
from rclpy.node import Node

SPI_SPEED = 2000000


class SpiBridge(Node):
    def __init__(self):
        super().__init__('spi_bridge')

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = SPI_SPEED
        """Update frequency
        Single targets: {baudrate} / ({uart frame length} * {cmd length} * {number of joint})
        = 115200 / (9 * 4 * 18) ~= 177Hz <- autodetect-baudrate on maestro
        = 200000 / (9 * 4 * 18) ~= 300Hz <- fixed-baudrate on maestro
        Multiple targets: {baudrate} / ({uart frame length} * ({header length} + {mini-cmd length} * {number of joints}))
        = 115200 / (9 * (3 + 2 * 18)) ~= 320Hz 
        = 200000 / (9 * (3 + 2 * 18)) ~= 560Hz"""

        self.subscription = self.create_subscription(
            UInt8MultiArray, 'stm32_cmd', self.callback, 10
        )

    def callback(self, msg: UInt8MultiArray):
        now = self.get_clock().now()

        data = bytearray(msg.data)
        self.spi.xfer3(data)

        elapsed = self.get_clock().now() - now
        print(f"Sent {len(data)} bytes in {elapsed.nanoseconds / 10**9:.3f}s")


def main(args=None):
    rclpy.init(args=args)

    node = SpiBridge()

    print('SPI Bridge ready!')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
