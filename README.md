# Dog Factory ROS 2 Humble + Gazebo

## 1. Objectif du projet

Ce workspace simule un robot-chien quadrupède dans une grande usine industrielle. Le robot possède un châssis, quatre jambes, une tête, une caméra RGB et un **lidar 2D**. Il peut avancer en autonomie, détecter un obstacle devant lui, choisir une direction de contournement, recevoir des commandes clavier et déclencher une trajectoire de saut via un service ROS 2.

> **Important :** ce projet est un démonstrateur pédagogique. Le contrôleur actuel illustre l’architecture ROS 2 et la publication de commandes, mais il ne remplace pas un contrôleur quadrupède de niveau industriel utilisant ros2_control, une estimation d’état, une cinématique inverse complète, un générateur de trajectoire et un contrôleur dynamique.

## 2. Architecture générale

| Package | Fonction | Fichiers importants |
|---|---|---|
| `dog_factory_description` | Description physique du robot | `urdf/dog.urdf.xacro` |
| `dog_factory_gazebo` | Monde et objets de l’usine | `worlds/factory.world` |
| `dog_factory_control` | Autonomie, téléopération et saut | `autonomy_node.py`, `keyboard_teleop.py`, `jump_controller.cpp` |
| `dog_factory_bringup` | Démarrage de tous les composants | `launch/factory_sim.launch.py` |

Le flux principal est le suivant : Gazebo calcule les rayons du lidar, le plugin ROS publie un message `sensor_msgs/msg/LaserScan` sur `/scan`, le nœud Python lit ce message et publie une vitesse de déplacement sur `/cmd_vel`. Le nœud C++ écoute le service `/dog/jump` et publie une trajectoire articulaire sur `/dog/joint_trajectory`.

## 3. Prérequis

Le projet cible Ubuntu 22.04 avec ROS 2 Humble et Gazebo Classic. Installez les dépendances avec :

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-tf2-ros ros-humble-geometry-msgs \
  ros-humble-sensor-msgs ros-humble-trajectory-msgs ros-humble-std-srvs \
  build-essential python3-colcon-common-extensions python3-rosdep
```

Initialisez ensuite l’environnement :

```bash
source /opt/ros/humble/setup.bash
cd ~/dog_factory_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. Compilation

Depuis la racine du workspace :

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Pour rendre le sourcing permanent dans Bash :

```bash
echo 'source ~/dog_factory_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 5. Démarrage de la simulation

Lancez Gazebo, le monde industriel, le robot, le robot state publisher, le lidar, la caméra, l’autonomie Python et le contrôleur de saut C++ avec :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py
```

Le robot apparaît approximativement en `(x=-4, y=0, z=0.8)`. Le monde comprend un sol, une rampe orange, une barrière, une rangée de caisses et deux piliers.

Pour lancer Gazebo sans interface graphique, utile sur une machine distante :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=false
```

## 6. Lidar : description détaillée

### 6.1 Emplacement physique

Le lidar est défini dans `src/dog_factory_description/urdf/dog.urdf.xacro`. Le fichier crée un lien nommé `lidar_link`, puis le fixe au châssis avec le joint `lidar_fixed`. Le support est placé à `z=0.28 m` par rapport à `base_link`.

### 6.2 Paramètres du capteur

| Paramètre | Valeur | Signification |
|---|---:|---|
| Type | `ray` | Capteur de rayons Gazebo |
| Nombre de mesures | `360` | Une mesure par degré |
| Champ de vision | `-pi` à `+pi` | Tour complet |
| Portée minimale | `0.12 m` | Les objets plus proches sont ignorés |
| Portée maximale | `12.0 m` | Distance maximale détectable |
| Fréquence | `15 Hz` | Nombre de scans par seconde |
| Message ROS | `sensor_msgs/msg/LaserScan` | Format standard ROS 2 |
| Topic | `/scan` | Topic consommé par l’autonomie |
| Frame | `lidar_link` | Repère du capteur |

Le plugin `libgazebo_ros_ray_sensor.so` transforme les rayons Gazebo en message ROS 2. Le remapping `~/out:=scan` donne le topic final `/scan` dans le namespace global du robot.

### 6.3 Vérification du lidar

Dans un terminal séparé :

```bash
source /opt/ros/humble/setup.bash
source ~/dog_factory_ws/install/setup.bash
ros2 topic list | grep scan
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Pour afficher les données dans RViz2 :

```bash
rviz2
```

Ajoutez un affichage **LaserScan**, sélectionnez le topic `/scan`, puis définissez le **Fixed Frame** sur `base_link` ou `odom` selon les TF disponibles.

## 7. Algorithme d’évitement

Le fichier `autonomy_node.py` divise les mesures du lidar en trois secteurs : gauche, avant et droite. La distance minimale de chaque secteur est calculée en ignorant les valeurs infinies. Si la distance frontale devient inférieure à `safe_distance`, le robot s’arrête longitudinalement et tourne vers le secteur qui paraît le plus libre. Sinon, il avance à la vitesse `cruise_speed`.

Le comportement est déterministe : le lidar publie un scan, le callback `scan_cb` mémorise le dernier scan, la boucle de contrôle s’exécute à 10 Hz, le secteur avant est comparé à la distance de sécurité, puis une commande `Twist` est publiée sur `/cmd_vel`.

## 8. Commandes ROS utiles

| Action | Commande |
|---|---|
| Voir tous les topics | `ros2 topic list` |
| Inspecter le lidar | `ros2 topic echo /scan` |
| Vérifier la fréquence lidar | `ros2 topic hz /scan` |
| Inspecter les vitesses | `ros2 topic echo /cmd_vel` |
| Appeler le saut | `ros2 service call /dog/jump std_srvs/srv/Trigger {}` |
| Vérifier les services | `ros2 service list` |
| Voir les nœuds | `ros2 node list` |
| Voir les connexions | `rqt_graph` |
| Arrêter le robot | `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'` |

## 9. Téléopération clavier

Exécutez :

```bash
ros2 run dog_factory_control keyboard_teleop
```

Les touches sont `w` pour avancer, `s` pour reculer, `a` pour tourner à gauche, `d` pour tourner à droite, `x` pour publier un arrêt et `q` pour quitter. Évitez de faire fonctionner simultanément l’autonomie et la téléopération, car deux nœuds pourraient publier des commandes concurrentes sur `/cmd_vel`.

## 10. Contrôleur de saut C++

Le fichier `jump_controller.cpp` crée le service `/dog/jump`. Une requête génère trois points de trajectoire : position accroupie, extension rapide et position d’atterrissage. Les positions concernent les articulations de genou et de cheville des quatre jambes.

```bash
ros2 service call /dog/jump std_srvs/srv/Trigger {}
```

La trajectoire est publiée sur `/dog/joint_trajectory`. Dans cette version pédagogique, un plugin d’interface articulaire complet n’est pas encore ajouté. Pour obtenir un saut physique réellement exécuté par Gazebo, il faut connecter cette trajectoire à `ros2_control` ou à des contrôleurs d’effort/position Gazebo.

## 11. Description du modèle Xacro

Le modèle est construit avec les éléments suivants :

| Élément | Rôle |
|---|---|
| `base_link` | Châssis principal |
| `body_shell` | Coque supérieure décorative |
| `head_link` | Tête articulée |
| `lidar_link` | Support du lidar |
| `camera_link` | Support de caméra |
| `*_hip` | Segment de hanche |
| `*_thigh` | Segment supérieur de jambe |
| `*_shin` | Segment inférieur de jambe |
| `*_foot` | Patte de contact avec le sol |

La macro Xacro `leg` évite de répéter manuellement la même structure pour les quatre jambes. Les propriétés visuelles utilisent des primitives box, cylinder et materials afin de ne pas dépendre de meshes propriétaires.

## 12. Monde usine

Le fichier `factory.world` contient des modèles SDF statiques. Le sol mesure `40 x 24 m`. La barrière, la rampe, les caisses et les piliers constituent des obstacles pour tester le lidar et l’évitement. Le moteur physique ODE utilise un pas de simulation de `0.001 s`.

## 13. Paramètres

Les paramètres principaux sont dans `dog_factory_bringup/config/controllers.yaml` :

| Paramètre | Valeur par défaut | Effet |
|---|---:|---|
| `cruise_speed` | `0.65` | Vitesse avant en m/s |
| `safe_distance` | `1.0` | Distance déclenchant l’évitement |
| `jump_height` | `0.55` | Valeur descriptive pour le scénario de saut |

Vous pouvez modifier ces valeurs, reconstruire si nécessaire, puis relancer le launch.

## 14. Organisation des fichiers

```text
 dog_factory_ws/
 ├── README.md
 ├── scripts/demo_commands.sh
 └── src/
     ├── dog_factory_description/
     │   └── urdf/dog.urdf.xacro
     ├── dog_factory_gazebo/
     │   └── worlds/factory.world
     ├── dog_factory_control/
     │   ├── dog_factory_control/autonomy_node.py
     │   ├── dog_factory_control/keyboard_teleop.py
     │   └── src/jump_controller.cpp
     └── dog_factory_bringup/
         ├── launch/factory_sim.launch.py
         └── config/controllers.yaml
```

## 15. Dépannage

Si Gazebo affiche `Failed to load plugin libgazebo_ros_ray_sensor.so`, vérifiez que `ros-humble-gazebo-ros-pkgs` est installé et que `/opt/ros/humble/setup.bash` est bien sourcé. Si `/scan` n’existe pas, vérifiez que le robot a été inséré après le lancement de Gazebo et inspectez les logs de `spawn_entity.py`.

Si le robot ne bouge pas, vérifiez que `/cmd_vel` reçoit des messages et que l’autonomie Python est active. Si une autre source publie simultanément sur `/cmd_vel`, arrêtez-la pour éviter les commandes concurrentes.

Si RViz ne montre pas le lidar, choisissez un frame fixe publié dans TF et ajoutez manuellement l’affichage `LaserScan` sur `/scan`.

## 16. Commentaires dans le code

Les fichiers principaux sont annotés avec des commentaires explicatifs en français. Les commentaires décrivent les imports, les paramètres, les topics, les callbacks, les conversions d’angles, les décisions d’évitement et la génération des trajectoires. Les fichiers XML/SDF sont documentés par sections afin de conserver une bonne lisibilité.

Une annotation littéralement différente sur chaque ligne rend toutefois les fichiers difficiles à maintenir. La version documentée privilégie donc des commentaires précis par instruction ou par bloc logique, sans ajouter du bruit répétitif sur les lignes purement syntaxiques.

## 17. Évolutions recommandées

Pour passer d’un démonstrateur à une simulation quadrupède plus réaliste, ajoutez `ros2_control`, des transmissions et des contrôleurs d’articulations, un modèle d’état IMU, une odométrie correcte, une cinématique inverse, un planificateur de pas, une navigation Nav2 et un contrôleur dynamique tel que MPC ou Whole-Body Control.

## Références

[1]: https://docs.ros.org/en/humble/ ROS 2 Humble Documentation
[2]: https://classic.gazebosim.org/tutorials Gazebo Classic Tutorials
[3]: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html ROS 2 URDF Tutorials
[4]: https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html `sensor_msgs/msg/LaserScan`
[5]: https://docs.ros.org/en/humble/p/launch/ ROS 2 Launch Documentation
