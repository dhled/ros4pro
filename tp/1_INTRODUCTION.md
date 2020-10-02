# ROS4PRO : Journée d'Introduction

## 1. Images des cartes SD
📥 Téléchargez ces images d'origine en cas de besoin de remettre à zéro les cartes SD :
* [Image du Turtlebot](http://www.robotis.com/service/download.php?no=1738)
* [Image originale de Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr/releases/download/2.0.0/2017-04-06-poppy-ergo-jr.img.zip) (avec l'interface graphique `http://poppy.local` mais sans ROS)
* [Image avec ROS de l'Ergo Jr](https://github.com/poppy-project/poppy_controllers/releases/download/v1.0/poppy-ergo-jr-ros-melodic.shrink2.img.zip)

### Comment flasher une carte SD avec une nouvelle image ?
📥 Télécharger l'image à flasher puis l'extraire du zip le cas échéant.
Ensuite, sur Ubuntu, taper la commande suivante en remplaçant le bon nom de fichier dans if (input file) :
```bash
sudo  dd  if=<FILE.img>  of=/dev/mmcblk0 bs=4M status=progress  
```
⏏️ Pour éjecter la carte SD, prenez soin de taper ensuite la commande suivante afin d'éviter de corrompre la copie :
```bash
sync
```

## 2. Documentation

## 3. Travaux pratiques
### 3.1. Personnalisation des noms de robots et ordinateurs

Au démarrage du cours, tous les robots et les ordinateurs possèdent le même nom à savoir `ubuntu` (votre ordinateur), `poppy` (le robot manipulateur), `turtlebot` (le robot roulant), ce qui posera problème lorsqu'on les fera communiquer ensemble. Pour ces 3 machines, nous allons donc changer leur nom, en ajoutant juste votre numéro de groupe à la fin, par exemple `poppy5`.

💻🤖 Pour personnaliser votre nom, il faut ouvrir un terminal sur la machine à renommer (via SSH pour les robots) puis :
```bash
sudo hostnamectl set-hostname <NOUVEAU_NOM>
```

### 4. FAQ Robots
#### Procédure de debug :

💻 Dans un terminal taper `ping poppy.local` (pour Poppy) ou `ping raspberrypi.local` (pour Turtlebot) :

  * **Si 1 ligne s'affiche chaque seconde** avec des statistiques de temps en millisecondes ➡️ Test réseau réussi. Vous avez peut-être oublié de démarrer le roscore ou bien `ROS_MASTER_URI` dans le fichier `~/.bashrc` pointe vers le mauvais robot
  * **Si une erreur survient** et la commande s'arrête ➡️ Test réseau échoué. Vérifiez que la LED verte ACT de la Raspberry Pi vacille pendant environ 45 secondes lorsque vous venez de brancher l'alimentation :
    * **Si `ACT` vacille** en 🟢 ➡️ Votre Raspberry Pi démarre correctement mais la configuration réseau est incorrecte. Vérifiez que vous avez placé le fichier `wpa_supplicant.conf` au bon endroit dans la partition `boot` sur la carte SD si vous êtes en Wifi ; ou bien connectez-vous avec un câble RJ45 sur un routeur
    * **Si `ACT` ne vacille pas** ➡️ Votre Raspberry Pi ne démarre pas correctement. La LED rouge `PWR` s'allume-t-elle ?
      * **Si `PWR` s'allume** en 🔴 ➡️ Votre Raspberry Pi est fonctionnelle mais la carte SD ne possède pas une iamge valable. Recommencez la procédure de flash ci-dessus.
      * **Si `PWR` ne s'allume pas** ➡️ Votre Raspberry Pi  n'est pas fonctionnelle. Vous avez peut-être mal branché la Pixl (Poppy) ou bien le câble rouge-noir (Turtlebot)

#### Connecter le robot en Wifi :
Insérer la carde SD du robot en question dans votre poste de travail et ouvrir la partition nommée `boot`, y télécharger le fichier [wpa_supplicant.conf](files/wpa_supplicant.conf) en indiquant le bon mot de passe wifi à l'intérieur mais sans changer son nom.

Taper la commande `sync` avant de retirer la carte SD. Ce fichier sera supprimé au démarrage du robot, signalant que la demande de connexion Wifi a bien été prise en compte.
