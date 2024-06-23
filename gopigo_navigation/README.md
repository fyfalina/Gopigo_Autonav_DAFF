# Package GoPiGo Navigation

Ce package implémente la logique de navigation qui permet au robot de se déplacer à l'intérieur du labyrinthe et de retrouver la sortie sans heurter les murs à l'aide de 3 capteurs ultrasons et l'odométrie fournis par le package `bringup_car`.

## Contenu

Vous y trouverez un seul script `auto_nav_DAFF.py`. 

## Prérequis

- ROS Noetic
- Python 3
- Package `gopigo3` installé et configuré

## Installation

Assurez-vous que le package `gopigo_navigation` est placé dans votre workspace ROS et que vous avez exécuté `catkin_make` et `source devel/setup.bash`.

## Utilisation

Pour lancer le script, utilisez la commande suivante :

```sh
rosrun gopigo_navigation auto_nav_DAFF.py


