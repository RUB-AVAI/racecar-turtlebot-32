# About
This Github Repository is part of the Autonomous Vehicles and Artificial Intelligence Lab Course WS 24/25. 

Autonomous driving is the future of individual mobility and all major manufacturers are working on fully autonomous vehicles. While there are robust and good solutions for the individual problems in autonomous driving, the main challenge lies in their integration. Altogether, an autonomous vehicle’s software is the biggest problem. Therefore, the key in self-driving vehicles is about getting the software right. In this course, we will investigate the different aspects of self-driving vehicles as well as the importance and application of artificial intelligence in this domain from a theoretical but especially from a practical perspective.

# Installation
ROS2 Humble must already be set up.

```
pip3 install -r requirements.txt
```

- Afterwards make sure "numpy<2". Otherwise downgrade manually

Make sure only one matplotlib version (pip version) is installed:

```
sudo apt remove python3-matplotlib
```

~~```sudo apt-get install ros-humble-tf-transformations```~~ 
- Don't need this anymore. We don't have sudo on the racecar, so we could not install the package. Instead we copied the file with the needed functions to our repo.

If using GUI in WSL: 
```
sudo apt install x11-apps -y
```

## Install torch
```
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

To use the GPU for Object detection, we need special prebuild wheels to work with the jetson. Find them [[here](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)]. Install them with:

```
pip install PACKAGENAME.whl --force-reinstall
```
You also may need some version of the package `onnxruntime-gpu`.
# Docker
Start: `docker compose up -d`

Stop: `docker compose down`

# Colcon build and install
```
colcon build

source install/setup.bash

colcon build --packages-select ackermann_msgs realsense2_camera_msgs vesc_msgs`urg_node_msgs
```
The folder `msg_stuff` contains packages to provide the message types used by the racecar (AckermannDrive messages, Camera images, etc.) and should be reusable in other projects with the racecar.
# Copy files script

Use the following script to update Files faster by directly copying them to sitepackages (20s colcon build vs 0.5s copy).
To use it you have to have ran `colcon build` at least once.
Make sure file endings are LF
```
./copyFilesToSitepackages.bash
```
# Execute Ros2 Nodes
## Run packages
```
ros2 run package_name node_name
```
## Launch file to run multiple nodes in one terminal
```
ros2 launch gui_pkg launch.py
```
## Run all
Use a new terminal for each command:
```
ros2 launch gui_pkg launch.py
ros2 run controller controller
# Optional:
ros2 run gui_pkg gui_node
```
Note that when using the GUI on Windows with WSL2, VSCode terminals don't work. You need the actual WSL2 terminal to have the GUI displayed on your Laptop.

## Other commands
```
source install/local_setup.bash

ros2 bag play /workspace/rosbag/avai1.rosbag
ros2 bag play --loop -r 0.6 /workspace/rosbag/avai1.rosbag
ros2 topic echo
ros2 topic list
```

## Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/VUXPDdpeL6c/0.jpg)](https://www.youtube.com/watch?v=VUXPDdpeL6c)

[Youtube Video](https://www.youtube.com/watch?v=VUXPDdpeL6c)
