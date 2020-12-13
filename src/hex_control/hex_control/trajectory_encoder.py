from math import pi
from typing import Dict, List

"""
import serial
import sys
import binascii

# cmd = '#1P1500T1000#2P1500T1000#3P1500T1000#4P1500T1000#5P1500T1000#6P1500T1000#7P1500T1000#8P1500T1000#9P1500T1000#10P1500T1000#11P1500T1000#12P1500T1000\r\n'
servo = int(sys.argv[1])
duty = int(sys.argv[2])
port = serial.Serial('/dev/ttyS0')
port.baudrate = 115200

duty *= 4
mask = 2**7 - 1  # 7 bits
low = duty & mask
high = int(duty / (2**7)) & mask
byts = bytearray([0x84, servo, low, high])
print([0x84, servo, low, high])


print(port.write(byts))
# print(port.write(byts))

"""




















class ServoConfig:
    def __init__(self, center: int, dir_: int):
        self.pin = 99
        self.center = center
        self.dir_ = dir_

    def conn(self, pin: int):
        self.pin = pin
        return self


class TrajectoryEncoder:
    MULTIPLE_TARGETS_CMD = 0x9F

    joint_to_servo = {
        'coxa_joint_l1': ServoConfig(2280, -1).conn(9),
        'femur_joint_l1': ServoConfig(1960, 1).conn(10),
        'tibia_joint_l1': ServoConfig(600, -1).conn(11),
        'coxa_joint_l2': ServoConfig(2150, -1).conn(12),
        'femur_joint_l2': ServoConfig(1860, 1).conn(13),
        'tibia_joint_l2': ServoConfig(580, -1).conn(14),
        'coxa_joint_l3': ServoConfig(2540, -1).conn(21),
        'femur_joint_l3': ServoConfig(1800, 1).conn(22),
        'tibia_joint_l3': ServoConfig(610, -1).conn(23),
        'coxa_joint_r1': ServoConfig(810, -1).conn(6),
        'femur_joint_r1': ServoConfig(1130, -1).conn(7),
        'tibia_joint_r1': ServoConfig(2450, 1).conn(8),
        'coxa_joint_r2': ServoConfig(1020, -1).conn(15),
        'femur_joint_r2': ServoConfig(1130, -1).conn(16),
        'tibia_joint_r2': ServoConfig(2400, 1).conn(17),
        'coxa_joint_r3': ServoConfig(380, -1).conn(18),
        'femur_joint_r3': ServoConfig(1080, -1).conn(19),
        'tibia_joint_r3': ServoConfig(2450, 1).conn(20),
    }

    def __init__(self):
        pass

    def encode_trajectory(self, trajectory: List[Dict[str, float]]) -> bytearray:
        result = bytearray([])

        for point in trajectory:
            result.extend(self.encode_step(point))

        return result

    def encode_step(self, angles: Dict[str, float]) -> bytearray:
        angles = [(joint, angle) for joint, angle in angles.items()]
        angles = sorted(angles, key=lambda x: self.joint_to_servo[x[0]].pin)

        lowest_pin = self.joint_to_servo[angles[0][0]].pin
        # Setting multitarget, servos MUST be on subsequent pins
        # Protocol: 0x9F, num of target, first channel number, cmds...
        cmd = bytearray([self.MULTIPLE_TARGETS_CMD, len(angles), lowest_pin])

        for joint, angle in angles:
            servo_config = self.joint_to_servo[joint]
            duty = self.angle_to_duty(servo_config, angle)

            cmd += self.make_cmd(servo_config.pin, duty)

        return cmd

    def make_cmd(self, servo_pin: int, duty: int):
        mask = 2**7 - 1  # last 7 bits
        duty *= 4
        low = duty & mask
        high = int(duty / (2**7)) & mask
        return bytearray([low, high])

    def angle_to_duty(self, servo_config: ServoConfig, angle: float):
        displacement = int(angle * 1000. / (pi / 2.)) * servo_config.dir_
        duty = servo_config.center + displacement
        if duty < 0 and duty + 4000 <= 2550:
            duty += 4000    # reverse direction
        duty = max(duty, 460)
        duty = min(duty, 2550)
        return duty
