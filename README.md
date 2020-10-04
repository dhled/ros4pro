# 🤖 Workshop ROS4PRO 🦾

Ce dépôt contient les ressources du workshop ROS4PRO pour l'enseignement de ROS en français 🇫🇷. Vous êtes sur la version exploitant les robots **Poppy Ergo Jr, Turtlebot, et la bibliothèque Keras**. Pour consulter les autres versions, changez de branche.

## [Accéder aux Travaux Pratiques](https://github.com/ros4pro/ros4pro/tree/poppy_tb3_keras/tp#travaux-pratiques)

## Utilisation
Il est recommandé d'utiliser la [clé USB Live](https://github.com/ros4pro/ros4pro/releases) où toutes les dépendances sont installées. Pour utiliser ces ressources dans votre propre environnement ROS vous trouverez des indicactions ci-dessous.

### System dependencies
The following command lines will install system-wide dependencies.
This is intended to run in Python 2 since this is still the default for ROS Melodic.
```
sudo apt install ros-melodic-moveit ros-melodic-realtime-tools ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-urdf-geometry-parser ros-melodic-gazebo-ros-pkgs ros-melodic-control-toolbox ros-melodic-gazebo-ros-control ros-melodic-turtlebot3-msgs ros-melodic-gmapping ros-melodic-move-base ros-melodic-amcl ros-melodic-map-server ros-melodic-dwa-local-planner python-wstool git tree python-pip

pip update -U pip

pip install tensorflow keras imageio matplotlib scikit-image numpy
```

### Git dependencies
The following command lines will download git repositories into your ROS workspace.
```
cd ~/catkin_ws/src
git clone https://github.com/ros4pro/ros4pro.git
wstool init
wstool merge ros4pro/.rosinstall
wstool update
```

### Build
As a reflex, build and source your workspace after installing new packages.
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
### Troubleshooting
#### Time querying and syncing
```
ntpdate -q SAWYER.local
sudo ntpdate -s ntp.ubuntu.com
```

#### Pip install on Live sticks
```
TMPDIR=/media/ubuntu/usbdata/ sudo -E pip install --cache-dir=/media/ubuntu/usbdata/ --build /media/ubuntu/usbdata/ scikit-image torch matplotlib tqdm torchvision visdom imageio

```

#### Live sticks shouldn't be all named the same `ubuntu.local` if `ROS_MASTER_URI` is used
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
