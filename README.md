# Workshop ROS4PRO

## 1. Install
### 1.1 Prerequisites
This is intended to run in Python 2 since this is still the default for ROS Melodic.
```
sudo apt install ros-melodic-moveit ros-melodic-realtime-tools ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-urdf-geometry-parser ros-melodic-gazebo-ros-pkgs ros-melodic-control-toolbox ros-melodic-gazebo-ros-control python-wstool git tree python-pip

pip install scikit-image torch matplotlib tqdm torchvision visdom imageio
```

### 1.2 Download dependencies
```
cd ~/catkin_ws/src
git clone https://github.com/ros4pro/ros4pro.git
wstool init
wstool merge ros4pro/.rosinstall
wstool update

cd ~/catkin_ws
catkin_make
```

## 2. Train the neural network
```
rosrun ros4pro learn.py
```

## 3A. Run in light simulation
For faster runs, Sawyer is not simulated through Gazebo but with fake motor controllers in RViz.

```
roslaunch ros4pro manipulate.launch
```

## 3B. Run on an actual Sawyer robot
For faster runs, Sawyer is not simulated through Gazebo but with fake motor controllers in RViz.

```
roslaunch ros4pro manipulate.launch simulation:=False
```

## 2. Troubleshooting
Time syncing:
```
sudo ntpdate -s ntp.ubuntu.com
```
