# sudo chgrp spi /sys/class/gpio/export
# sudo chmod 660 /sys/class/gpio/export
# sudo chgrp spi /sys/class/gpio/unexport
# sudo chmod 660 /sys/class/gpio/unexport
# sudo chgrp spi /dev/spidev0.0
# sudo chmod 660 /dev/spidev0.0

import sys

import spidev


servo = int(sys.argv[1])
duty = int(sys.argv[2])

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 3000000
# spi.lsbfirst = False
# spi.mode = 0b11

spi.xfer([42])

duty *= 4
mask = 2**7 - 1  # 7 bits
low = duty & mask
high = int(duty / (2**7)) & mask
byts = bytearray([0x9F, 1, servo, low, high])

resp = spi.xfer(byts)
print(resp)
