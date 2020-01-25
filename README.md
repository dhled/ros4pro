# Workshop ROS4PRO
🇫🇷 Ce dépôt contient les réponses (le code) du challenge d'intégration Turtlebot + Sawyer du workshop ROS4PRO 

🇬🇧 This package hosts answers (code) of the Turtlebot + Sawyer integration challenge of the ROS4PRO workshop

### 1. System dependencies
The following command lines will install system-wide dependencies.
This is intended to run in Python 2 since this is still the default for ROS Melodic.
```
sudo apt install ros-melodic-moveit ros-melodic-realtime-tools ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-urdf-geometry-parser ros-melodic-gazebo-ros-pkgs ros-melodic-control-toolbox ros-melodic-gazebo-ros-control ros-melodic-turtlebot3-msgs ros-melodic-gmapping ros-melodic-move-base ros-melodic-amcl ros-melodic-map-server python-wstool git tree python-pip

pip install tensorflow keras imageio matplotlib scikit-image numpy
```

### 2. Git dependencies
The following command lines will download git repositories into your ROS workspace.
```
cd ~/catkin_ws/src
git clone https://github.com/ros4pro/ros4pro.git
wstool init
wstool merge ros4pro/.rosinstall
wstool update
```

### 3. Build
As a reflex, build and source your workspace after installing new packages.
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 4. Train the neural network
```
rosrun ros4pro learn.py
```

## 5. Create the map
TODO: gmapper and teleop

## 6. Serve the map and navigate with RViz
TODO

## 7A. Run manipulation and navigation nodes in light simulation
For faster runs, Sawyer is not simulated through Gazebo but with fake motor controllers in RViz.
The following commands in 2 different terminals will run the background services for manipulation and navigation and also run  the `manipulate.py` and `navigate.py` scenarii.
```
roslaunch ros4pro manipulate.launch
roslaunch ros4pro navigate.launch
```

## 7B. Run on an actual Sawyer robot
For faster runs, Sawyer is not simulated through Gazebo but with fake motor controllers in RViz.

```
roslaunch ros4pro manipulate.launch simulation:=False
```

## 8. Troubleshooting
Time syncing:
```
sudo ntpdate -s ntp.ubuntu.com
```

Pip install on Live sticks:
```
TMPDIR=/media/ubuntu/usbdata/ sudo -E pip install --cache-dir=/media/ubuntu/usbdata/ --build /media/ubuntu/usbdata/ scikit-image torch matplotlib tqdm torchvision visdom imageio

```
