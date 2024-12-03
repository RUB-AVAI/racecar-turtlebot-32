# Docker


## Start
docker compose up -d

## Stop
docker compose down

# Rosbag

source install/local_setup.bash

ros2 bag play

ros2 bag play --loop -r 0.1 /workspace/rosbag/avai1.rosbag

ros2 topic echo

ros2 topic list

# install Numpy

pip install "numpy<2"

pip install -r /workspace/src/requirements.txt


# install torch CPU

pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
