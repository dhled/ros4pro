# ROS4PRO : Journée d'Introduction

## 1. Images des cartes SD
Téléchargez ces images d'origine en cas de besoin de remettre à zéro les cartes SD :
* [Image du Turtlebot](http://www.robotis.com/service/download.php?no=1738)
* [Image originale de Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr/releases/download/2.0.0/2017-04-06-poppy-ergo-jr.img.zip) (avec l'interface graphique `http://poppy.local` mais sans ROS)
* [Image avec ROS de l'Ergo Jr](https://github.com/poppy-project/poppy_controllers/releases/download/v1.0/poppy-ergo-jr-ros-melodic.shrink2.img.zip)

### Comment flasher une carte SD avec une nouvelle image ?
Télécharger l'image à flasher puis l'extraire du zip le cas échéant.
Ensuite, sur Ubuntu, taper la commande suivante en remplaçant le bon nom de fichier dans if (input file) :
```bash
sudo  dd  if=<FILE.img>  of=/dev/mmcblk0 bs=4M status=progress  
```

## 2. Documentation

## 3. Travaux pratiques
### 3.1. Personnalisation des noms de robots et ordinateurs

Au démarrage du cours, tous les robots et les ordinateurs possèdent le même nom à savoir `ubuntu` (votre ordinateur), `poppy` (le robot manipulateur), `turtlebot` (le robot roulant), ce qui posera problème lorsqu'on les fera communiquer ensemble. Pour ces 3 machines, nous allons donc changer leur nom, en ajoutant juste votre numéro de groupe à la fin, par exemple `poppy5`.

Pour personnaliser votre nom, il faut ouvrir un terminal sur la machine à renommer (via SSH pour les robots) puis :
```bash
sudo hostnamectl set-hostname <NOUVEAU_NOM>
```
