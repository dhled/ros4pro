# ROS4PRO: Journée Navigation

## 1. Documentation
[Documentation du TB3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) 

[gmapping](http://wiki.ros.org/gmapping)

[move_base](http://wiki.ros.org/move_base)

## 2. Travaux pratiques

### 2.0. Bringup du TB3 
Avant de lancer la cartograpgie ou la navigation vérifiez la configuration réseau de ROS sur votre PC et sur le TB3 (les valeurs de *ROS_MASTER_URI* et *ROS_HOSTNAME* dans le .bashrc).

Commencez par lancer un roscore sur votre PC ou le TB3, cela dépend de là valeur de *ROS_MASTER_URI*.

Sur le TB3 lancer la commande `roslaunch turtlebot3_bringup turtlebot3_robot.launch`.
S'il n'y a aucune erreur vous êtes prêt à lancer la cartographie puis la navigation autonome.

### 2.1. Cartographie 
Lancez le commande `roslaunch turtlebot3_slam turtlebot3_slam.launch`. RViz ce lance et vous devez voir le robot, les scans du LIDAR et la carte en construction.

Dans un nouveau terminal lancez la commande `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` et restez sur le terminal pour controler le robot avec le clavier.

Quand la carte est terminée **ne quittez ni RViz ni le terminal de la cartographie**, dans un nouveau terminal lancez la commande `roscd turtlebot3_navigation/maps/` pour aller dans le dossier où la carte est stockée. **Attention**, la commande qui va suivre va supprimer la carte précédente, faites en une copie si vous souhaitez la conserver. Lancez la commande `roslaunch ros4pro map_saver.launch` qui va sauvegarder la carte dans les fichiers maps.yaml et maps.pgm et écraser les anciens.

### 2.2. Navigation
Lancez le commande `roslaunch turtlebot3_navigation turtlebot3_navigation.launch` pour lancer la localisation et la navigation autonome.

Sur RViz vous devez voir le robot, les scans du LIDAR, les partiules de AMCL et la carte que vous avez faite.

Si le robot est mal localisé, utilisez l'outil *2D Pose Estimate* sur RViz. Cliquez et Glissez avec la souris pour positionner le robot sur la carte.

Pour donner des ordres de navigation, utilisez l'outil *2D Nav Goal* sur RViz. Cliquez et Glissez avec la souris sur la carte là où le robot doit aller.

### 2.3 TP de navigation
L'objectif final du TP est de faire passer le robot par une suite de points de passage, comme pour une patrouille.

Le noeud *navigation_scenario.py* réalise cette tâche mais vous devez lui apporter quelques modifications pour réussir le TP.

Pour exécuter le scénario lancez la navigation en suivant **2.2 Navigation** puis lancez la commande `rosrun ros4pro navigation_scenario.py`


