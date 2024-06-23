# Projet GoPiGo3 Navigation Autonome

Dans le cadre d'un projet pour le cours de Robotique, nous avons programmé un robot GoPiGo3 afin qu'il navigue de manière autonome à l'intérieur d'un labyrinthe et trouver le chemin de sortie sans aucune intervention externe. Ce repository contient les deux packages principaux, `gopigo3` et `gopigo_navigation`, utilisés pour implémenter cette naviguation autonome.

## Contenu du Repository

- `gopigo3` : Ce package contient les fonctions de base pour contrôler le robot GoPiGo3.
- `gopigo_navigation` : Ce package implémente les algorithmes de navigation pour permettre au robot de se déplacer de manière autonome en utilisant des capteurs ultrason et l'odométrie fournis par le package `gopigo3`.

Vous trouverez plus de détails sur le contenu et le fonctionnement de chaque package dans leurs Readme respectifs.

## Prérequis

- ROS Noetic
- Ubuntu 20.04
- GoPiGo3 Robot
- Capteurs ultrason pour le GoPiGo3
- Python 3

## Installation

Clonez ce repository et placez-le dans votre `~/catkin_ws/src`.

```sh
cd ~/catkin_ws/src
git clone https://github.com/votre-repository/gopigo3_navigation.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Utilisation

Pour lancer le projet, exécutez les commandes suivantes dans 2 terminaux différents :

### Lancer le contrôle de base du robot

```sh
roslaunch bringup_car differential_drives.launch
```

### Lancer la navigation autonome

```sh
rosrun gopigo_navigation auto_nav_DAFF.py
```

## Auteurs

- Adrien Ronan Norbert
- Diren Gokhool
- Feelminho Mahefaherivola
- Fy Falina Randrianarijaona
