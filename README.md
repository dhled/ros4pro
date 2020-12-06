# 🤖 Workshop ROS4PRO 🦾

Ce dépôt contient les ressources du workshop ROS4PRO pour l'enseignement de ROS en français 🇫🇷. Vous êtes sur la version exploitant les robots **Poppy Ergo Jr, Turtlebot, et la bibliothèque Keras**. Pour consulter les autres versions, changez de branche.

## [Accéder aux Travaux Pratiques](https://learn.ros4.pro/fr/)

## Installation

L'installation est déjà réalisée sur les clés USB Live fournies pour la formation. Si vous n'utilisez pas de clé, voici la procédure :
```bash
sudo apt install python3-rosinstall python3-rosdep
sudo rosdep init && rosdep update
cd ~/catkin_ws/src && wstool init && wstool merge https://raw.githubusercontent.com/ros4pro/ros4pro/poppy_tb3_keras/.rosinstall && wstool update
cd ~/catkin_ws/ && rosdep install --from-paths src --ignore-src -r -y
catkin_make && source ~/catkin_ws/devel/setup.bash
```

*This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No. 732287*.
