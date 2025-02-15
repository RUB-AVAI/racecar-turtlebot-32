# Docker


## Start
`docker compose up -d`

## Stop
`docker compose down`

# Rosbag
```
source install/local_setup.bash

ros2 bag play

ros2 bag play --loop -r 0.6 /workspace/rosbag/avai1.rosbag

ros2 topic echo

ros2 topic list
```

# Installation
ROS2 Humble must already be set up.

```
pip3 install -r requirements.txt # "numpy<2"
# make sure only one matplotlib version (pip version) is installed
sudo apt remove python3-matplotlib
# sudo apt-get install ros-humble-tf-transformations #DONT NEED THIS ANYMORE

# if using GUI in WSL
sudo apt install x11-apps -y

# install torch CPU
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

# Colcon build and install
```
colcon build

source install/setup.bash

# Use the following script to update Files faster by directly copying them to sitepackages (20s colcon build vs 0.5s copy)
# To use it you have to have ran "colcon build" at least once.
# make sure file endings are LF
./copyFilesToSizepackages.bash
```

# Run packages
```
ros2 run package_name node_name
```