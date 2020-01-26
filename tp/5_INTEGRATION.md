# ROS4PRO : Journée Intégration

## 1. Architecture

Le scenario est le suivant :

1. Le réseau de neurone a été entraîné au préalable sur les batchs MNIST avec `learn.py`
2. Sawyer prend une photo au dessus du feeder avec sa camera `right_hand_camera` (étape "scan")
3. Cette photo est envoyée au serveur de vision `vision_server.py` qui va :
  3.1. extraire les contours carrés des cubes
  3.2. transmettre les imagettes rognées selon leurs contours au réseau de neurones
  3.3. le réseau de neurone va effectuer une prédiction sur le label marqué à la main 1 ou 2
4. Le noeud de manipulation récupère les coordonnées des cubes et leur label
5. Pour chaque cube, il effectue un pick-and-place pour le positionner sur la remorque du Turtlebot. Le label est passé sur un paramètre `/ros4pro/label`
6. Le Turtlebot lit ce paramètre et se rend au conteneur 1 ou 2
7. Il effectue une rotation de 360° pour faire chuter le cube dans la zone de tri à l'aide du mât

### 2. Les points d'entrée 
#### 2.1. Côté navigation
Le noeud `integrated_navigate.py` est similaire au noeud `navigate_waypoints.py` utilisé le jour 2. Additionnellement, il attend le paramètre suivants de Sawyer avant de démarrer la navigation :

 * `/ros4pro/label`
 
Suivant la valeur du paramètre le robot va déposer le cube à un des deux points. Trouvez les coordonnées de ces points sur la carte et écrivez les dans le script *ros4pro/src/nodes/navigate_integrate.py*.
Après avoir déposé le cube, le robot va recommencer et attendre de recevoir une nouvelle valeur.

Ce noeud a besoin de la navigation : `roslaunch turtlebot3_navigation turtlebot3_navigation.launch`
Attendez que la navigation soit initialisée pour lancer le noeud : `rosrun ros4pro navigate_integrate.py`
 
#### 2.2. Côté manipulation
Pour la manipulation, nous utiliserons le même noeud `manipulate.py` que celui de la journée Manipulation. Lorsqu'on le démarre avec `roslaunch` on ajoutera l'argument `vision` pour lui indiquer qu'il doit faire appel au serveur de vision et donc au réseau de neurones plutôt que d'utiliser l'emplacement vert prédéfini. C'est à dire :
```
roslaunch ros4pro manipulate.launch simulate:=false vision:=true
```
