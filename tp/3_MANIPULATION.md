# ROS4PRO : Journée Manipulation

## 1. Documentation
### 1.1. Les liens
* [Tutoriaux de MoveIt](https://ros-planning.github.io/moveit_tutorials/)
* [Code du MoveIt Commander Python](https://github.com/ros-planning/moveit/tree/master/moveit_commander/src/moveit_commander)
* [Documentation de l’API MoveIt en Python](http://docs.ros.org/melodic/api/moveit_python/html/namespacemoveit__python.html)
* [Tutoriaux du SDK Sawyer](https://sdk.rethinkrobotics.com/intera/Tutorials)

## 1.2. Assembler Poppy Ergo Jr

Pour assembler votre robot, veuillez suivre [le guide d'assemblage](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/), en suivant les étapes faîtes pour ROS le cas échéant. 
Pour vérifier que votre assemblage est correct, connectez-vous en SSH au robot (si ce n'est pas déjà fait) puis exécutez :
```
ssh pi@poppy.local      # password raspberrypi
# Effacer éventuellement l'ancienne clé ECDSA si vous avez un message d'erreur
roslaunch poppy_controllers control.launch
```
Vous devriez voir apparaître `Connection successful`. Si l'erreur `"Connection to the robot can't be established"` est affichée, alors votre robot n'a pas été monté correctement. La suite de ce message d'erreur indique quel(s) moteur(s) pose(nt) problème pour vous aider à le résoudre. Fermez avec Ctrl+C puis utilisez de nouveau Poppy Configure si un moteur est mal configuré.

**Remarque :** Si vos moteurs clignotent en rouge : votre code a créé une collision et ils se sont mis en alarme. Pour désactiver l'alarme il faut débrancher et rebrancher l'alimentation, ce qui fera aussi redémarrer le robot

## 2. Travaux pratiques
### 2.1. Comprendre la représentation d'un robot ROS

Un robot intégré à ROS est composé d'au minimum :
* un descripteur URDF
* un contrôleur qui gère les E/S avec le robot

#### 2.1.1. Comprendre le descripteur URDF

Clonez le package ROS Poppy Ergo Jr Description sur votre PC, il contient le fichier de description URDF du robot :
```
git clone https://github.com/poppy-project/poppy_ergo_jr_description.git
```

Compilez votre workspace puis sourcez votre `.bashrc`, enfin rdv dans le dossier `urdf` de ce package, puis exécutez la commande `urdf_to_graphiz` qui convertit un fichier URDF en représentation graphique dans un PDF :
```
sudo apt install liburdfdom-tools
roscd poppy_ergo_jr_description/urdf
urdf_to_graphiz poppy_ergo_jr.urdf
```

Ouvrez le PDF obtenu puis déterminez :
* Que représentent les rectangles ?
* Que représentent les bulles ?
* Que représentent les flèches et surtout les valeurs `xyz` et `rpy` associées ?

#### 2.1.2. Comprendre les E/S du contrôleur

Le contrôleur se trouve déjà sur le robot. Vous pouvez directement vous connecter au robot et le démarrer :

```
ssh pi@poppy.local      # password raspberrypi
# Effacer éventuellement l'ancienne clé ECDSA si vous avez un message d'erreur
roslaunch poppy_controllers control.launch
```

Sur votre PC, faîtes pointer votre `ROS_MASTER_URI` sur `poppy.local`. Rappel :
```
nano ~/.bashrc      # Pour changer votre ROS_MASTER_URI
source ~/.bashrc    # Pour charger votre .bashrc et donc le nouveau master
```

##### 2.1.2.a. Topics du robot
Avec l'utilitaire `rostopic`, lister les topics disponibles puis consultez celui qui décrit l'état courant des joints, en particulier :

* Quel est son nom ?
* Quel est le type de message qu'il transmet ?
* A quelle fréquence (en Hertz) est-ce qu'il met à jour l'état des joints ?

##### 2.1.2.b. Services du robot
Avec les utilitaires `rosservice` et `rossrv`, listez les services disponibles puis consultez celui qui met le robot en mode **compliant**. En particulier :

* Quel est son nom ?
* Quel est le type de service qu'il transmet ?
* Quels sont les champs de la requête de ce service ?
* Quels sont les champs de la réponse de ce service ?
* Appelez ce service pour activer et désactiver le mode compliant et essayer de faire bouger votre robot à la main à chaque fois. Que déduisez-vous de la signification du **mode compliant** ? *Conseil :* aidez-vous de l'autocomplétion avec la touche <TAB>

##### 2.1.2.c. Tracer la courbe des positions des moteurs en temps réel
Mettez votre robot en mode compliant, démarrez `rqt_plot` pour tracer les positions des 6 moteurs ... bougez les moteurs à la main et vérifiez que `rqt_plot` actualise la courbe en temps réel.

### 2.2. Cinématique, et planification avec MoveIt dans RViz
#### 2.2.1. Démarrer avec MoveIt
Installez MoveIt puis clonez le package ROS **Poppy Ergo Jr MoveIt Configuration**, il contient le code nécessaire pour que ce robot fonctionne avec MoveIt :
```
sudo apt install ros-melodic-moveit
git clone https://github.com/poppy-project/poppy_ergo_jr_moveit_config.git
```

Compilez votre workspace puis sourcez votre `.bashrc`. Démarrez MoveIt avec `roslaunch` avec le paramètre `fake_execution` à false pour se connecter au vrai robot :
```
roslaunch poppy_ergo_jr_moveit_config demo.launch fake_execution:=false
```
Rviz doit démarrer avec un Poppy Ergo Jr en visu.

Note : si vous devez passer en simulation à ce moment suite à un défaut matériel, pensez à changer votre `ROS_MASTER_URI` pour `localhost` puis mettre simplement `fake_execution` à `true`.

#### 2.2.2. Planification

Dans l'onglet Planning, section **Query** puis **Planning group**, sélectionnez le groupe `arm_and_finger`, bougez le goal (la sphère 3D bleue) en position et en orientation puis cliquez sur **Plan**. Trois représentations 3D de robots se superposent, déterminez le rôle de chacun d'entre eux en testant également la fonctionnalité **Plan and Execute** :

* Que désigne le robot gris parfois mobile mais lent ?
* Que désigne le robot orange (fixe) ?
* Que désigne le robot gris qui répète infiniment un mouvement rapide ?
* Dans RViz, activer l'affichage du modèle de collision dans `Displays`, `Scene Robot`, `Show Robot Collision`, quelle est la forme de ce modèle utilisé par OMPL pour simplifier le calcul des collisions ?

#### 2.2.3. Planning groups
Testez également le groupe `arm` en plus du premier `arm_and_finger` et lancez des planifications de mouvement pour tester :

* Quelle est la différence entre ces 2 groupes ?
* Quel est le groupe pour lequel le goal est le plus facilement manipulable et pourquoi ?
* Déduisez-en ce que désigne exactement un `planning group`

#### 2.2.4. Transformations `tf`
Nous allons visualiser et interroger l'arbre des transformations nommé `tf`

Démarrer MoveIt puis dans un autre terminal lancer `rosrun tf view_frames`. Un fichier PDF a été créé, les `frames` (repères) qu'ils contient sont les mêmes que ceux dessinés par Rviz en rouge-vert-bleu.

* Comment est nommé le repère de base ?
* Comment sont nommés les deux effecteurs finaux possibles ?
* Avec `rosrun tf tf_echo`, déterminez quelle est la position actuel d'un effecteur dans le repère de base. Ses coordonnées peuvent vous servir pour les définir comme cible à atteindre par la suite.

### 2.3. Ecrire un noeud Python ROS pour l'Ergo Jr
#### 2.3.1. Créer un nouveau package et un nouveau noeud Python
Nous allons créer un nouveau package ROS nommé **ros4pro_custom** sur votre laptop de développement, qui contient notre code:
```
cd ~/catkin_ws/src
catkin_create_pkg ros4pro_custom          # Cette commande créé le package
mkdir -p ros4pro_custom/src                  # On créé un dossier src dans le package
touch ros4pro_custom/src/manipulate.py       # On créé un noeud Python "manipulate.py"
chmod +x ros4pro_custom/src/manipulate.py    # On rend ce noeud exécutable pour pouvoir le lancer avec rosrun
```

Bien que vous devriez avoir compris comment créer un noeud ROS en Python dans les tutoriels d'introduction, voici un rappel de noeud ROS minimal qui boucle toutes les secondes en Python :
```
#!/usr/bin/env python

import rospy

rospy.init_node('ros4pro_custom_node')
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rospy.loginfo("Hello world from our new node!")
    rate.sleep()
```


Compilez votre workspace puis sourcez votre `.bashrc`. Exécutez votre noeud avec rosrun :
```
cd ~/ros_ws
catkin_make
rosrun ros4pro_custom manipulate.py
```

Votre noeud doit afficher un message toutes les secondes, vous pouvez le tuer avec Ctrl+C. Nous allons ajouter du code petit à petit. Attention à l'ajouter au bon endroit pour créer un script cohérent.

#### 2.3.2. Planifier et exécuter des mouvements avec MoveIt

Le `MoveGroupCommander` est le commandeur de robot de MoveIt, il suffit de lui indiquer quel est le nom du groupe à commander puis donner une cible et appeler la fonction `go()` pour l'atteindre en évitant les obstacles. Cette cible peut être dans l'espace cartésien ou dans l'espace des joints :

##### 2.3.2.a. Cible dans l'espace cartésien

```
from moveit_commander.move_group import MoveGroupCommander
commander = MoveGroupCommander("arm_and_finger")
commander.set_pose_target([0.00, 0.079, 0.220] + [0.871, -0.014, 0.079, 0.484])
commander.go()
```

Les coordonnées cartésiennes de la cible sont les coordonnées de l'effecteur (càd `moving_tip` pour le groupe `arm_and_finger` ou bien `fixed_tip` pour le groupe `arm`) dans le repère `base_link`, exprimées sous la forme `x, y, z, qx, qy, qz, qw`.

##### 2.3.2.b. Cible dans l'espace des joints (sans évitement de collision)

Il est également possible de définir une cible dans l'espace des joints en fournissant une liste des 6 angles moteurs  dans ce cas il n'y a pas d'évitement de collision:

```
commander.set_joint_value_target([0, 0, 0, 0, 0, 0])
commander.go()
```

##### 2.3.2.c. Mise en pratique

* A l'aide des fonctions et commandes vues en 3.1.4. et 4.2.1., vérifiez que vous savez prendre les coordonnées cartésiennes courante et les définir comme cible puis l'atteindre
* A l'aide des fonctions et commandes vues en 2.2.1. et 4.2.2., vérifiez que vous savez prendre les positions des joints courantes et les définir comme cible puis l'atteindre
* A l'aide du mode compliant, prendre les coordonnées cartésiennes de l'effecteur et et les positions des joints pour deux configurations différentes du robot A et B (e.g. effecteur vers le haut et effecteur vers le bas)
* Faîtes bouger le robot infiniement entre les cibles cartésiennes A et B, nous y ajouterons des obstacles plus tard

#### 2.3.3. Déclarer des obstacles
Afin que les algorithmes de planification de trajectoire d'OMPL (tels que `RRTConnect`) puissent éviter les obstacles, il est nécessaire que MoveIt ait connaissance de leur position et leur forme. Il est possible d'utiliser une caméra de profondeur (aka caméra RGB-D, mais nous n'en avons pas ici) ou bien déclarer les objets depuis le code Python grâce à l'interface `PlanningSceneInterface`. Par exemple, ce code déclarer une boite de céréales comme objet de collision en spécifiant sa position et son orientation sous forme d'objet `PosteStamped` ainsi que sa taille en mètres :

```
from geometry_msgs.msg import PoseStamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface

scene = PlanningSceneInterface()
rospy.sleep(1)

ps = PoseStamped()
ps.header.frame_id = "base_link"
ps.pose.position.x = 0.15
ps.pose.position.y = 0
ps.pose.position.z = 0.15
ps.pose.orientation.w = 1
scene.add_box("boite_de_cereales", ps, (0.08, 0.24, 0.3))

rospy.sleep(1)
```

Les coordonnées des objets de collision sont données sous la forme d'objet `PoseStamped` incluant la `position`, l'`orientation` et le repère `frame_id`, et la taille est donnée sous forme de tuple (longueur, largeur, hauteur).

* Modifier l'obstacle "boite_de_cereales" proposé en exemple afin qu'un obstacle viennent perturber le mouvement entre les deux poses de votre programme en 3.2.2. et vérifiez que MoveIt contourne toujours ces obstacles sans jamais les collisionner.

**Note**: Accessoirement, il est possible d'attacher et de détacher les objets de collision au robot, ceci permet par exemple de simuler la saisie et la dépose d'objets physique dans RViz avec MoveIt. cf [la documentation MoveIt pour Python](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html) ou même [le code de `PlanningSceneInterface`](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_commander/src/moveit_commander/planning_scene_interface.py#L56)

#### 2.3.4. Enregistrer et rejouer un mouvement de pick-and-place
Référez-vous à la documentation du [Poppy Controllers](https://github.com/poppy-project/poppy_controllers/#3-trajectory-record-and-playback-feature) afin d'enregistrer et de rejouer des mouvements en utilisant la compliance du robot. Faîtes quelques essais avec plusieurs mouvements qui s'alternent pour bien comprendre le fonctionnement.

* Enregistrez un mouvement de pick-and-place pour attraper un cube et le déposer à un autre endroit

#### 2.4. Récupérer les images de la caméra en Python

Avec la carte SD ROS, l'image de la caméra est accessible par appel d'un service dédié. Nous aurons besoin de récupérer le package Poppy Controllers et le compiler d'abord :

```
cd ~/ros_ws/src
git clone https://github.com/poppy-project/poppy_controllers.git    # Nous aurons besoin de ce package
cd ~/ros_ws/
catkin_make
source ~/.bashrc
```

Testez ce code pour vérifier que vous pouvez récupérer l'image en Python via le service ROS `/get_image` fourni par le contrôleur.

```
import cv2
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

get_image = rospy.ServiceProxy("get_image", GetImage)
response = get_image()
bridge = CvBridge()
image = bridge.imgmsg_to_cv2(response.image)
cv2.imshow("Poppy camera", image)
cv2.waitKey(200)

```
Cette image peut ensuite être traitée par un réseau de neurones, une fonction OpenCV, etc ...

## 3. A rendre

* Réponses aux questions ci-dessus dans un fichier README (réponses communes au groupe mais répondez-ici ensemble pour vous assurer que tout le monde a compris)
* Code de votre pacakge `ros4pro_custom` à rendre



























### 1.2. Le plus important
Voici les quelques fonctions les plus importantes traduites depuis la documentation.

#### 1.2.1. Déclarer un commandeur de robot 
```
commander = MoveGroupCommander("right_arm")
```
Ce commandeur est celui qui identifie notre robot et permettra d'en prendre le contrôle.

#### 1.2.2. Exécuter un mouvement vers une cible
Définir une cible, dans l'espace des joints :
```
commander.set_joint_value_target([-1.0, -2.0, 2.8, 1.5, 0.1, -0.3, 3.0])
```
Les 7 valeurs sont les angles cibles des 7 joints en radians
Attention : les cibles dans l'espace des joints n'auront pas d'évitement de collisions

Ou bien définir une cible dans l'espace cartésien :
```
commander.set_pose_target([0.5, 0.05, 1.1, 0, 0, 0, 1])
```
Les 7 valeurs sont la **position** et l'*orientation* [**x, y, z**, *qx, qy, qz, qw*] cible de l'effecteur dans le repère `base`

#### 1.2.3. Planifier & exécuter le mouvement vers la cible
La fonction "go" déclenche le calcul de trajectoire et l'exécution instantanée si le calcul a réussi :
```
commander.go()
```

### 1.2.4. Exécuter une trajectoire cartésienne
Par opposition à la cible cartésienne, dans cet exemple au lieu de ne définir qu'une cible finale, on demande à MoveIt de suivre une trajectoire rectiligne dans l'espace cartésien. Cette trajectoire est précalculée en entier grâce à la fonction `commander.compute_cartesian_path([pose1, pose2]), resolution, jump)`
où :
* `[pose1, pose2]` est la liste des points à suivre de manière rectiligne de type `geometry_msgs.Pose`
* `resolution` est la distance en mètre entre 2 points cartésiens générés (par exemple `0.01` pour générer un point tous les centimètres)
* `jump` est le seuil maximal autorisé au delà duquel un saut trop important entre 2 positions angulaires en radians ne sera pas exécutée car il demanderait une vitesse excessive. `jump` est la somme des seuils sur tous les joints (par exemple `3.14`).

La fonction `commander.compute_cartesian_path(...)` renvoie :
* `trajectory`: la trajectoire cartésienne calculée
* `ratio`: Un ratio entre 0 et 1 indique la quantité de la trajectoire qui a pu être calculée avec succès sans générer de saut. Un ratio inférieur à 0.95 signifie probablement que la trajectoire est inutilisble car elle ne suit pas le chemin demandé. 

Par exemple, étant données 2 points `p1` et `p2` de type `geometry_msgs.Pose`, cet appel est valide :
```
trajectory, ratio = commander.compute_cartesian_path([pose1, pose2]), 0.01, 3.14)
```

Enfin, exécuter la trajectoire, uniquement si le ratio indique au moins 95% de succès :
```
commander.execute(trajectory)
```

#### 1.2.5. Définir des objets de collision
Lorsqu'on ajoute des objets de collision, les appels à `go()` dans l'espace cartésien planifieront, si possible, une trajectoire en évitant les objets déclarés comme objets de collision.

La scène de planification est notre interface pour ajouter ou supprimer des objets de collision :
```
scene = PlanningSceneInterface()
```
On peut ensuite effectuer les ajouts ou suppressions. Par exemple, on ajoute un objet de collision cubique de taille 10x8x2 cm à la position [1.2, 0.5, 0.55] et avec l'orientation [0, 0, 0, 1] (= rotation identité) dans le repère `base`:  

```
ps = PoseStamped()
ps.header.frame_id = "base"
ps.pose.position.x = 1.2
ps.pose.position.y = 0.5
ps.pose.position.z = 0.55
ps.pose.orientation.x = 0
ps.pose.orientation.y = 0
ps.pose.orientation.z = 0
ps.pose.orientation.w = 1
scene.add_box("ma_boite", list_to_pose_stamped2([[1.2, 0.5, 0.55], [0, 0, 0, 1]]), (0.10, 0.08, 0.02))
```
Les objets de collision apparaissent en vert dans RViz s'ils sont définis correctement.

Note: après une modification de la scène, est généralement utile de faire une pause `rospy.sleep(1)` afin de laisser le temps à la scène d'être mise à jour sur tout le système avant toute nouvelle planification de trajectoire

## 3. Travaux pratiques
### 3.1. Utiliser MoveIt dans le visualisateur Rviz
Avec roslaunch, lancer `sawyer_moveit.launch` provenant du package `sawyer_moveit_config`:
```
roslaunch sawyer_moveit_config sawyer_moveit.launch
```

Via l’interface graphique, changer l’orientation et la position de l’effecteur puis demander à MoveIt de planifier et exécuter une trajectoire pour l’atteindre.

* Cochez la bonne réponse : Cette méthode permet-elle de définir une cible dans l’espace : ◻ cartésien ◻ des joints

* Trois robots semblent superposés en 3D, quelles sont leurs différences :
  * Le robot orange est ... ?
  * Le robot rapide est ... ?
  * Le robot lent est ... ?

* Utilisez `rostopic echo` pour afficher en temps réel les messages du topic `/robot/joint_states`. Exécutez un mouvement et observer les valeurs changer. Que représente le topic /robot/joint_states ? 

* Indiquez comment se nomment les 7 joints de Sawyer depuis la base jusqu’à l’effecteur :
  * Premier joint :
  * Deuxième joint :
  * Troisième joint :
  * Quatrième joint :
  * Cinquième joint :
  * Sixième joint :
  * Dernier joint :

### 3.2. Utiliser MoveIt via son client Python
Dans le package `ros4pro`, ouvrir le nœud `manipulate.py`. Repérez les 3 exemples :
* d'exécution d'une trajectoire cartésienne
* de planification vers une cible cartésienne
* de planification vers une cible dans l'espace des joints

Durant la suite du TP, nous démarrerons notre programme de manipulation avec le launchfile `manipulate.launch` du package `ros4pro` qui fonctionne par défaut en mode simulé, à savoir :

```
roslaunch ros4pro manipulate.launch
```
Celui-ci démarre automatiquement `manipulate.py`, il est donc inutile de le démarrer par un autre moyen.

#### 3.2.1. Modifier la scène de planification
La scène représente tout ce qui rentre en compte dans les mouvements du robot et qui n’est pas le robot lui-même : les obstacles et/ou les objets à attraper. Ces éléments sont déclarés à MoveIt comme des objets de collision (détachés du robot ou attachés c’est-à-dire qu’ils bougent avec lui).

* Prenez les mesures du feeder puis déclarez-les comme objets de collision dans votre noeud Python via l’interface `PlanningSceneInterface`. Complétez le TODO associé à la question 3.2.1. dans `manipulate.py`.

* Planifiez et exécutez un mouvement RViz pour vérifier que les collisions sont évitées. Déclarez un nouvel obstacle qui entrave forcément le chemin du robot et vérifiez. Vérifiez ce qu’il se passe lorsque le planner ne trouve aucune solution.

#### 3.2.2. Effectuer un pick-and-place avec un cube simulé (optionnel)
Nous considérons un cube situé à la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ` `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ = [[0.32, 0.52, 0.32], [1, 0, 0, 0]]`, ce qui correspond exactement à l'emplacement entouré en vert sur le feeder.

Pour l’approche, on positionnera le gripper 18cm au dessus du cube le long de son axe z.

* Sachant cela, déduire la matrice de transformation `ᶜᵘᵇᵉP₉ᵣᵢₚₚₑᵣ` en notation `[[x, y, z], [x, y, z, w]]` ?
* Exprimez `ᵇᵃˢᵉP₉ᵣᵢₚₚₑᵣ` en fonction de la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ` du cube dans le repère `base` via une multiplication matricielle

Ensuite, dans votre code :

* Inventez un cube en simulation dans votre code à la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ = [[0.32, 0.52, 0.32], [1, 0, 0, 0]]` c’est-à-dire à la surface du feeder. Pour ce faire utilisez :
  * l’interface `PlanningSceneInterface` pour ajouter, supprimer un cube ou l’attacher à l’effecteur `right_gripper_tip` du robot
  * `TransformBroadcaster` pour publier la frame tf nommée `cube` au centre du cube à attraper

#### 3.2.3. Générer les 4 trajectoires du pick-and-place
Pour rappel, voici les 4 étapes d'un pick pour attraper et relacher le cube simulé :
    1. trajectoire d’approche : aller sans collision à `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ` c'est à dire 18cm au dessus du cube sur son axe z (axe bleu)
    2. trajectoire de descente : suivre une trajectoire cartésienne de 50 points descendant le long de l'axe z pour atteindre  `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ`  avec l’effecteur `right_gripper_tip`. Puis fermer l'effecteur.
    3. trajectoire de retraite : retourner au point d’approche par une trajectoire cartésienne
    4. trajectoire de dépose : si le cube a bel-et-bien été attrapé avec succès, aller sans collision au point de dépose `ᵇᵃˢᵉP꜀ₔₑₚₒₛₑ = [[0.5, 0, 0.1], [0.707, 0.707, 0, 0]]`

* Dans `manipulate.py`, les deux dernières trajectoires sont incomplètes : retraite et dépose nommées `release` et `place`. Compléter les 2 TODO associés à la question 3.2.3 dans `manipulate.py`

* Vérifier que votre pick-and-place a l'air correct **en simulation d'abord** et que vous distinguez correctement **les 4 étapes du pick-and-place**.

### 3.3. Exécutez le pick-and-place sur le Sawyer réel
* Changez votre rosmaster pour celui de Sawyer :
```
export ROS_MASTER_URI=http://021608CP00013.local:11311
```
* Le centre du cube `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ` précédent est celui de la zone verte sur le feeder. Vérifiez que votre pick-and-place fonctionne avec 1 seul cube à l’emplacement vert.

* Pour commander le robot réel modifiez le paramètre `simulate` :
```
roslaunch ros4pro manipulate.launch simulate:=false
```

Vous devriez constater que votre pick-and-place fonctionne parfois et qu'il échoue dans certaines situations.

### 3.4. Préparer le pipeline du scenario final (optionnel)
Ajoutez à `manipulate.py` le code nécessaire pour votre scenario final :
* **Prise de photo (scan)** :
  * positionner l’effecteur de telle manière que  « right_hand_camera » se trouve à la verticale du feeder
  * Récupérer l’image rectifiée sur le topic dédié :
    * image_view peut aider à visualiser l’image pour déboguer dans un terminal
    * pour récupérer l’image en Python, implémentez un Subscriber. Cette photo sera envoyée au réseau de neurones lorsqu’il sera implémenté
* **Pick-and-Place successifs** : Effectuez 3 pick-and-place successifs sans intervention humaine  de 3 cubes dont la position est connue à l’avance. Ces positions seront retournées par le réseau de neurones lorsqu’il sera implémenté
* Si vous n’avez pas de collision de manière reproductible, vous pouvez accélérer les vitesses dans sawyer_moveit_config/config/joint_limits.yaml (pas les accélérations)
        
### 3.5. Prévenir les échecs de planification
Le path-planning étant réalisé pendant l’exécution et non précalculé, il est possible que MoveIt ne trouve aucune solution de mouvement. Pour remédier à cela, plusieurs pistes s’offrent à nous :
* Réessayer encore et encore … bêtement, jusqu’à un potentiel succès
* Planifier toute les trajectoires d’un coup avant exécution, si l’une ne peut être calculée, regénérer la précédente et recommencer. Cela montre ses limites : et si le robot n’est pas à l’emplacement attendu entre deux trajectoires, par exemple si l’opérateur l’a bougé manuellement ?
* **Fournir une « seed » à l’IK ou au path-planning**. Cette solution est approfondie ci-après :

MoveIt possède un système de calcul de la **géométrie directe et inverse**, respectivement via les services `/compute_fk` et `/compute_ik`.
Observer les types de service (`rosservice info`) et le contenu de la requête (`rossrv show`) puis appeler ces deux services sur ces deux exemples :
* Calculer la FK pour les angles `-π/2, π/2, -π/2, 0, 0, 0, 0`. Donner le résultat au format `[[x, y, z,], [qx, qy, qz, qw]]`
* Calculer l’IK pour la position d’effecteur `[[0.5, 0, 0.1], [0.707, 0.707, 0, 0]]`. Donner le résultat au format `[angle1, angle2, angle3, angle4, angle5, angle6, angle7]`

Réexécutez le même calcul d’IK sur la même position d’effecteur, une seconde puis une troisième fois. Observez que le résultat est différent. Pourquoi ?

* Fournissez une « seed » au format `[angle1, angle2, angle3, angle4, angle5, angle6, angle7]` de votre choix à l’IK pour influencer le résultat.
* Quelle seed proposez-vous pour maximiser les chances de succès du path-planning ?
