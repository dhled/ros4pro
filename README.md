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
wget -qO- https://raw.githubusercontent.com/ros4pro/ros4pro/poppy_tb3_keras/.bashrc | tee --append ~/.bashrc
catkin_make && source ~/.bashrc
```

## Mise à jour
Voici la procédure pour mettre à jour votre code pendant les TP :
```bash
cd ~/catkin_ws/src
wstool update
```

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
