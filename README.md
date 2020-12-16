# Workshop ROS4PRO
🇫🇷 Ce dépôt contient les réponses (le code) du challenge d'intégration Turtlebot + Sawyer du workshop ROS4PRO 

🇬🇧 This package hosts answers (code) of the Turtlebot + Sawyer integration challenge of the ROS4PRO workshop

### 1. System dependencies
The following command lines will install system-wide dependencies.
This is intended to run in Python 2 since this is still the default for ROS Melodic.
```
sudo apt install ros-melodic-moveit ros-melodic-realtime-tools ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-urdf-geometry-parser ros-melodic-gazebo-ros-pkgs ros-melodic-control-toolbox ros-melodic-gazebo-ros-control ros-melodic-turtlebot3-msgs ros-melodic-gmapping ros-melodic-move-base ros-melodic-amcl ros-melodic-map-server ros-melodic-dwa-local-planner python-wstool git tree python-pip

pip update -U pip

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
### Time querying and syncing
```
ntpdate -q SAWYER.local
sudo ntpdate -s ntp.ubuntu.com
```

### Pip install on Live sticks
```
TMPDIR=/media/ubuntu/usbdata/ sudo -E pip install --cache-dir=/media/ubuntu/usbdata/ --build /media/ubuntu/usbdata/ scikit-image torch matplotlib tqdm torchvision visdom imageio

```

### Live sticks shouldn't be all named the same `ubuntu.local` if `ROS_MASTER_URI` is used
Pour résoudre le problème de noms identiques sur les Live USB de manière définitive :

1. `nano ~/.bashrc`
2. Descendre tout en bas
3. Repérer la ligne qui exporte `ROS_HOSTNAME` et la supprimer définitivement
4. Ajouter à la place cette ligne :
```
export ROS_IP=`ip address|grep inet|grep dynamic|tr ' ' ':'|cut -d':' -f6|cut -d'/' -f1|head -n1`
```
5. Enregistrer avec Ctrl+X
6. Fermer et rouvrir les terminaux (ce qui déclencher un rechargement du fichier .bashrc)


***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No. 732287.
