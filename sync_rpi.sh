echo "Syncing to RPi..."
rsync -a bildros ubuntu@192.168.1.25:~/hex/
rsync -a clear_ros ubuntu@192.168.1.25:~/hex/
rsync -a sor ubuntu@192.168.1.25:~/hex/
rsync -a --exclude=__pycache__ src/ ubuntu@192.168.1.25:~/hex/src
