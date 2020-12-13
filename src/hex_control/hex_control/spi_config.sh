sudo chgrp spiuser /sys/class/gpio/export
sudo chmod 660 /sys/class/gpio/export
sudo chgrp spiuser /sys/class/gpio/unexport
sudo chmod 660 /sys/class/gpio/unexport
sudo chgrp spiuser /dev/spidev0.0
sudo chmod 660 /dev/spidev0.0
