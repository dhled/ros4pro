# ROS4PRO : Journée Manipulation

## 1. Documentation
TODO

## 2. La théorie
### 2.1. Déclarer un commandeur de robot 
```
commander = MoveGroupCommander("right_arm")
```
### 2.2. Exécuter un mouvement vers une cible
#### 2.2.1. Définir une cible, dans l'espace des joints ou bien cartésien
```
commander.set_joint_value_target([-1.0, -2.0, 2.8, 1.5, 0.1, -0.3, 3.0])
```
Les 7 valeurs sont les angles cibles des 7 joints en radians
Attention : les cibles dans l'espace des joints n'auront pas d'évitement de collisions

Ou bien dans l'espace cartésien :
```
commander.set_pose_target([0.5, 0.05, 1.1, 0, 0, 0, 1])
```
Les 7 valeurs sont la **position** et l'*orientation* [**x, y, z**, *qx, qy, qz, qw*] cible de l'effecteur dans le repère `base`

#### 2.2.2. Planifier & exécuter le mouvement vers la cible
```
commander.go()
```

### 2.3. Exécuter une trajectoire cartésienne
#### 2.3.1 Précalculer la trajectoire cartésienne
Au lieu de ne définir qu'une cible finale, on demande à MoveIt de suivre une trajectoire rectiligne dans l'espace cartésien. Cette trajectoire est précalculée en entier grâce à la fonction `commander.compute_cartesian_path([pose1, pose2]), resolution, jump)`
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

#### 2.3.2. Exécuter la trajectoire
A ne faire que si le ratio indique au moins 95% de succès :
```
commander.execute(trajectory)
```

## 3. Travaux pratiques
