# ROS4PRO : Journée d'Introduction

## 1. Images des cartes SD
Téléchargez ces images d'origine en cas de besoin de remettre à zéro les cartes SD :
* [Image du Turtlebot](http://www.robotis.com/service/download.php?no=1738)
* [Image originale de Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr/releases/download/2.0.0/2017-04-06-poppy-ergo-jr.img.zip) (avec l'interface graphique `http://poppy.local` mais sans ROS)
* [Image avec ROS de l'Ergo Jr](https://github.com/poppy-project/poppy_controllers/releases/download/v1.0/poppy-ergo-jr-ros-melodic.shrink2.img.zip)

### Comment flasher une carte SD avec une nouvelle image ?
```bash
sudo  dd  if=<FILE.img>  of=/dev/mmcblk0 bs=4M status=progress  
```

## 2. Documentation

## 3. Travaux pratiques
