# Projet GoPiGo3 Navigation Autonome

Ce repository contient deux packages principaux, `gopigo3` et `gopigo_navigation`, utilisés pour programmer un robot GoPiGo3 afin qu'il navigue de manière autonome à l'intérieur d'un labyrinthe.

## Contenu du Repository

- `gopigo3` : Ce package contient les fonctions de base pour contrôler le robot GoPiGo3.
- `gopigo_navigation` : Ce package implémente les algorithmes de navigation pour permettre au robot de se déplacer de manière autonome en utilisant des capteurs ultrason et l'odométrie fournis par le package `gopigo3`.

## Prérequis

- ROS Noetic
- Ubuntu 20.04
- GoPiGo3 Robot
- Capteurs ultrason pour le GoPiGo3
- Python 3

## Installation

Clonez ce repository et placez-le dans votre espace de travail ROS (généralement `~/catkin_ws/src`).

```sh
cd ~/catkin_ws/src
git clone https://github.com/votre-repository/gopigo3_navigation.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
