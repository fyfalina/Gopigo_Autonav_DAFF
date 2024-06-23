# Package GoPiGo Navigation

Ce package implémente l'algorithme de navigation qui permet au robot de se déplacer à l'intérieur du labyrinthe et de retrouver la sortie sans heurter les murs à l'aide de 3 capteurs ultrasons et l'odométrie fournis par le package `bringup_car`.

## Contenu

Ce package contient un script principal, `auto_nav_DAFF.py`, qui :
- Interprète les signaux envoyés par les capteurs ultrasoniques.
- Gère la logique de navigation autonome pour permettre au robot d'éviter les obstacles et ajuster sa direction en conséquence. 

### Fonctions clés

- `distance(TRIG_PIN, ECHO_PIN)` : Fonction qui mesure la distance par rapport aux murs du labyrinthe à l'aide des capteurs ultrason.
- `turn_safely(direction, max_angle=math.pi/2)` : Fonction pour effectuer des rotations exactes en utilisant les données d'odométrie.
- `back_and_reassess()` : Fonction pour reculer et réévaluer la situation lorsque toutes les directions sont bloquées.

## Fonctionnalités

- Navigation autonome à l'aide de capteurs ultrason placés à l'avant et sur les côtés.
- Détection d'obstacles et prise de décision pour éviter les collisions.
- Utilisation de l'odométrie pour calculer les angles de rotation et garantir des mouvements précis.
- Algorithme de backtracking pour reculer et réévaluer la direction lorsque tous les côtés sont bloqués.

## Prérequis

- Package `bringup_car` installé et configuré

## Installation

1. Assurez-vous que le package `gopigo_navigation` est placé dans votre workspace ROS et que vous avez exécuté `catkin_make` et `source devel/setup.bash`.

2. Installez les dépendances requises :

    ```sh
    sudo apt-get install python3-rospy
    sudo apt-get install ros-noetic-geometry-msgs
    sudo apt-get install ros-noetic-nav-msgs
    ```

3. Assurez-vous que la bibliothèque RPi.GPIO est installée :

    ```sh
    sudo apt-get install python3-rpi.gpio
    ```

## Utilisation

Pour utiliser ce package, vous devez lancer le script principal qui gère la navigation autonome.

1. Assurez-vous que d'avoir lancé le fichier launch dans le package bringup_car :

    ```sh
    roslaunch bringup_car differential_drives.launch
    ```

2. Assurez-vous que le script de navigation est exécutable :

    ```sh
    chmod +x src/gopigo_navigation/auto_nav_DAFF.py
    ```
    
3. Lancez le script de navigation :

    ```sh
    rosrun gopigo_navigation auto_nav_DAFF.py
    ```



