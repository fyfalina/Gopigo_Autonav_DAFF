# Package bringup_car

Le package `bringup_car` est conçu pour gérer les opérations de base du robot GoPiGo3, y compris le contrôle des moteurs et la publication de l'odométrie pour la navigation. Ce package est essentiel pour initialiser le robot et préparer l'environnement ROS avant de lancer des tâches de navigation complexes.

## Contenu

Le package contient un script principal `differential_drives.py` qui initialise le robot GoPiGo3 et configure les abonnements et publications ROS nécessaires pour son fonctionnement.

### Script : `differential_drives.py`

- **Initialisation du robot** : Configure le GoPiGo3 pour les opérations et initialise les GPIO pour la gestion de l'alimentation.
- **Gestion des moteurs** : Configure les moteurs pour la commande de vitesse et de position.
- **Publication de l'odométrie** : Publie les données d'odométrie sur le topic `/odom`.
- **Services ROS** : Fournit des services pour réinitialiser les moteurs et gérer l'alimentation.

### Fonctions clés

- **`odometry(self, left, right)`** : Calcule l'odométrie basée sur les encodeurs des moteurs, ce qui est essentiel pour la navigation autonome.
- **`on_twist(self, twist)`** : Convertit les commandes de mouvement en signaux pour les moteurs, permettant un contrôle précis de la direction et de la vitesse du robot.

## Fonctionnalités

- **Initialisation des GPIO** : Configuration des GPIO pour la gestion de l'alimentation du GoPiGo3.
- **Abonnement et publication ROS** : Configure les abonnements aux topics pour les commandes de moteurs et les servos, ainsi que la publication de l'état des moteurs et de l'odométrie.
- **Services de gestion** : Fournit des services pour réinitialiser les moteurs et gérer l'alimentation, facilitant ainsi le contrôle à distance.

## Prérequis

- Bibliothèque `RPi.GPIO` pour la gestion des GPIO

## Installation

Assurez-vous que le package `bringup_car` est placé dans votre workspace ROS et que vous avez exécuté `catkin_make` et `source devel/setup.bash`.

1. Installez les dépendances requises :

    ```sh
    sudo apt-get install python3-rospy
    sudo apt-get install ros-noetic-geometry-msgs
    sudo apt-get install ros-noetic-nav-msgs
    sudo apt-get install python3-rpi.gpio
    ```

2. Assurez-vous que le script `differential_drives.py` est exécutable :

    ```sh
    chmod +x src/bringup_car/differential_drives.py
    ```

## Utilisation

Pour utiliser ce package, vous devez lancer le script principal via un fichier launch.

1. Assurez-vous que le ROS Master est en cours d'exécution :

    ```sh
    roscore
    ```

2. Lancez le fichier launch pour démarrer le script :

    ```sh
    roslaunch bringup_car differential_drives.launch
    ```

   Ce fichier launch démarre le script `differential_drives.py` qui configure et initialise le GoPiGo3 pour une utilisation ultérieure avec d'autres packages, tels que `gopigo_navigation`.
