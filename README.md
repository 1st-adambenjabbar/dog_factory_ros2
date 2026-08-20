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

## 19. Nœud C++ de perception LiDAR

Le nœud `lidar_obstacle_detector` réalise une perception temps réel directement à partir du topic `/scan`. Il ne s’agit pas d’une caméra : dans ce contexte, la vision par ordinateur demandée est interprétée comme une perception géométrique du milieu à partir du LiDAR 2D.

Le nœud applique quatre étapes. Il filtre les valeurs invalides, convertit chaque rayon polaire en coordonnées cartésiennes, regroupe les points voisins en clusters et calcule le centre moyen de chaque cluster. Les clusters contenant moins de `min_cluster_points` mesures sont rejetés afin de réduire les faux obstacles issus du bruit.

### Compilation et lancement

Le nœud est automatiquement compilé par `dog_factory_control` et démarré par :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py
```

Pour le lancer séparément après compilation :

```bash
ros2 run dog_factory_control lidar_obstacle_detector
```

### Topics publiés

| Topic | Type | Utilisation |
|---|---|---|
| `/lidar/obstacles` | `geometry_msgs/msg/PoseArray` | Centres des obstacles détectés |
| `/lidar/front_obstacle_distance` | `std_msgs/msg/Float32` | Distance minimale dans le cône frontal |
| `/lidar/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | Cylindres visualisables dans RViz2 |

### Paramètres du détecteur

| Paramètre | Valeur | Rôle |
|---|---:|---|
| `min_range` | `0.12` | Distance minimale conservée |
| `max_range` | `12.0` | Distance maximale conservée |
| `cluster_distance` | `0.35` | Écart maximal entre deux points d’un même obstacle |
| `min_cluster_points` | `3` | Nombre minimal de points par obstacle |
| `front_danger_distance` | `1.0` | Seuil de diagnostic frontal |

Les paramètres peuvent être inspectés avec :

```bash
ros2 param list /lidar_obstacle_detector
ros2 param get /lidar_obstacle_detector cluster_distance
ros2 param set /lidar_obstacle_detector cluster_distance 0.50
```

### Visualisation dans RViz2

Lancez `rviz2`, choisissez `base_link` ou `lidar_link` comme frame fixe, puis ajoutez un affichage **MarkerArray** sur `/lidar/obstacle_markers`. Les obstacles apparaissent comme des cylindres rouges centrés sur les clusters détectés. Vous pouvez aussi ajouter **PoseArray** sur `/lidar/obstacles` pour observer directement les centres géométriques.

### Vérification en ligne de commande

```bash
ros2 topic echo /lidar/obstacles
ros2 topic echo /lidar/front_obstacle_distance
ros2 topic echo /lidar/obstacle_markers
ros2 topic hz /lidar/obstacles
```

Le nœud est volontairement sans OpenCV, car les données d’entrée sont des distances angulaires et non des images. Pour une version plus avancée, il serait possible d’ajouter une caméra RGB, OpenCV et une fusion caméra-LiDAR ; le détecteur actuel constitue la branche de perception géométrique robuste et légère pour l’évitement.

## 20. Architecture séparée robot / usine

La description est maintenant séparée en deux responsabilités. Le package `dog_robot_description` contient le robot-chien, ses liens, ses articulations, son lidar et sa caméra. Le package `dog_factory_environment` contient uniquement le monde SDF, le sol, la rampe, la barrière, les caisses et les piliers.

Cette séparation permet de remplacer l’usine sans modifier le robot, ou de réutiliser le robot dans un autre monde. Le launch principal utilise désormais `dog_robot_description` et `dog_factory_environment`.

## 21. Machine à états de saut Python

Le fichier `dog_factory_control/dog_factory_control/jump_state_machine.py` fournit une alternative Python au contrôleur C++ historique. Ses états sont `IDLE`, `CROUCH`, `TAKEOFF`, `FLIGHT` et `LAND`. Chaque transition publie une posture articulaire sur `/dog/joint_trajectory`.

Pour demander un saut :

```bash
ros2 service call /dog/jump_python std_srvs/srv/Trigger {}
```

Pour autoriser un déclenchement automatique devant un obstacle :

```bash
ros2 param set /jump_state_machine auto_jump true
```

Le saut automatique utilise `/lidar/front_obstacle_distance`. Pour une exécution physique complète, il faut encore connecter le topic de trajectoire à un contrôleur d’articulations Gazebo ou `ros2_control`.

## 22. Fusion LiDAR + caméra

Le fichier `sensor_fusion_node.cpp` reçoit `/scan` et `/image_raw`. Le LiDAR fournit la distance géométrique frontale ; la caméra fournit un score visuel léger calculé sur la luminosité moyenne d’une fenêtre centrale. La fusion ne remplace pas la géométrie LiDAR : elle ajoute un score de confiance lorsque l’image est récente.

Les sorties sont `/perception/fused_obstacles` de type `geometry_msgs/msg/PoseArray` et `/perception/fusion_confidence` de type `std_msgs/msg/Float32`. Le nœud est lancé automatiquement par `factory_sim.launch.py`.

Cette implémentation évite une dépendance obligatoire à OpenCV et `cv_bridge`. Pour une perception industrielle avancée, remplacez le score de luminosité par une détection OpenCV ou un modèle neuronal, puis ajoutez une calibration extrinsèque caméra-LiDAR et une transformation TF.

## 23. Tests pytest

Les tests sont dans `dog_factory_control/tests/test_lidar_obstacle_detector.py`. Ils couvrent les scans vides, les obstacles proches, deux obstacles séparés, le rejet du bruit isolé, la distance frontale dangereuse et l’absence d’obstacle frontal.

Après compilation ROS 2 :

```bash
colcon test --packages-select dog_factory_control
colcon test-result --verbose
```

Les tests utilisent des nuages de points synthétiques et reproduisent l’algorithme de segmentation du C++. Ils permettent de valider les cas géométriques indépendamment de Gazebo.

## 24. Commentaires XML et déclaratifs

Les nouveaux fichiers `dog_robot_description/package.xml`, `dog_factory_environment/package.xml`, les CMakeLists, le monde SDF et le launch contiennent désormais des commentaires explicatifs par section. Le cœur Xacro est conservé séparément afin de préserver une description robot fonctionnelle et réutilisable ; `dog_robot.urdf.xacro` documente son rôle d’interface.

## 25. Navigation autonome avec Nav2

Le package `dog_factory_navigation` sépare la navigation de la description du robot et du monde usine. Il contient une carte d’occupation PGM, son fichier YAML, les paramètres Nav2 et `launch/navigation.launch.py`.

Nav2 utilise `/scan` pour AMCL et les costmaps, `/odom` pour le mouvement local, `map -> odom` pour la localisation et `/cmd_vel` pour commander le robot. Le planificateur global est NavFn et le contrôleur local est DWB.

### Lancement local complet sur Ubuntu 22.04 avec ROS 2 Humble

Installez les paquets nécessaires :

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-xacro \
  ros-humble-rviz2 ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-ament-cmake-pytest
```

Préparez et compilez le workspace :

```bash
source /opt/ros/humble/setup.bash
cd ~/dog_factory_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Lancez Gazebo seul :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=true navigation:=false
```

Lancez l’environnement complet avec Nav2 :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=true navigation:=true
```

Le lancement complet démarre Gazebo, le monde industriel, le robot-chien, le lidar, la caméra, l’autonomie, la perception, AMCL, la carte, le planificateur global, le contrôleur local et Behavior Tree Navigator.

### Visualisation Gazebo et RViz2

Gazebo s’ouvre automatiquement grâce à `gui:=true`. Pour afficher Nav2 dans RViz2 :

```bash
rviz2
```

Dans RViz2, utilisez `map` comme **Fixed Frame**, puis ajoutez `Map` sur `/map`, `LaserScan` sur `/scan`, `TF`, `RobotModel`, `Map` ou `Costmap` sur les topics `/local_costmap/costmap` et `/global_costmap/costmap`, ainsi que `Path` sur `/plan` et `/local_plan`.

Envoyez ensuite un objectif avec l’outil **2D Goal Pose**. En ligne de commande, l’action Nav2 peut être appelée ainsi :

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Vérifiez le fonctionnement avec :

```bash
ros2 node list
ros2 topic list | grep -E 'map|costmap|plan|cmd_vel|scan|odom'
ros2 topic echo /amcl_pose
ros2 topic echo /cmd_vel
```

### Remarque importante sur la dynamique quadrupède

Nav2 suppose qu’un contrôleur bas niveau transforme `/cmd_vel` en déplacement. Le workspace actuel conserve une base pédagogique et le topic `/cmd_vel` doit être relié à une locomotion quadrupède réelle pour obtenir une navigation physique exacte. La carte fournie est un point de départ ; pour une navigation industrielle, créez une carte SLAM avec `slam_toolbox` puis remplacez `factory.yaml`.

## 26. Hyperparamètres RL du saut

Le fichier `src/dog_factory_control/config/rl_jump.yaml` propose une configuration PPO de départ. Il ne lance pas automatiquement un entraînement RL : il sert de contrat de configuration pour un environnement Gymnasium/RLlib/SB3 qui devra communiquer avec Gazebo ou une simulation vectorisée.

| Groupe | Paramètres principaux | Conseil de réglage |
|---|---|---|
| Exploration | `initial_action_noise`, `final_action_noise` | Commencer haut puis réduire progressivement |
| PPO | `learning_rate`, `clip_range`, `update_epochs` | Réduire le learning rate si les sauts deviennent instables |
| Récompense | `reward_success`, `reward_failure`, `reward_energy_penalty` | La réussite doit dominer l’énergie, sans ignorer la stabilité |
| Curriculum | `curriculum_obstacle_height`, `curriculum_max_obstacle_height` | Augmenter la hauteur seulement après un taux de réussite stable |
| Dynamique | `target_clearance`, `target_forward_speed` | Mesurer la hauteur réelle et la vitesse après réception |
| Reproductibilité | `seed`, `num_envs` | Fixer la seed pour comparer deux expériences |

Une stratégie recommandée est de commencer avec des obstacles de 0.15 m, une pénalité de chute forte et une récompense de stabilité modérée. Lorsque le taux de réussite dépasse environ 80 % sur plusieurs milliers d’épisodes, augmentez progressivement la hauteur, la variation de masse, le bruit lidar et les perturbations de vitesse. N’augmentez pas simultanément toutes les difficultés, sinon il devient difficile d’identifier la cause d’une divergence.

La fonction de récompense devrait combiner la réussite du franchissement, la hauteur au-dessus de l’obstacle, la stabilité du corps, la vitesse horizontale après réception, l’énergie des articulations et les collisions. Une récompense uniquement basée sur la hauteur peut produire des sauts excessifs ; une récompense uniquement basée sur la vitesse peut produire des collisions.

Le pipeline RL conseillé est : entraînement hors ligne dans des épisodes courts, validation sur des obstacles jamais vus, randomisation de la masse et de la friction, puis transfert vers Gazebo avec bruit capteur et limites d’action. La machine à états Python reste le contrôleur déterministe de secours pendant l’expérimentation.

## 27. Dépannage Nav2

Si AMCL ne publie pas `map -> odom`, vérifiez que `/scan`, `/odom` et `/tf` existent, que `use_sim_time` est activé et que la carte est bien installée. Si le planificateur trouve un chemin mais que le robot ne bouge pas, inspectez `/cmd_vel` et le contrôleur bas niveau. Si la carte paraît inversée, vérifiez `origin`, `resolution`, `occupied_thresh` et `free_thresh` dans `factory.yaml`.

Si Nav2 n’est pas trouvé, installez `ros-humble-navigation2` et `ros-humble-nav2-bringup`, puis sourcez de nouveau `/opt/ros/humble/setup.bash` et `install/setup.bash`.

## 28. Réglage de la fusion LiDAR-caméra

Les paramètres du nœud `sensor_fusion_node` sont maintenant regroupés dans `src/dog_factory_control/config/sensor_fusion.yaml`. Le fichier est chargé automatiquement par le launch principal.

| Paramètre | Rôle | Effet d’une augmentation |
|---|---|---|
| `front_angle` | Demi-angle du cône LiDAR avant | Prend en compte une zone plus large |
| `obstacle_distance` | Distance de déclenchement LiDAR | Rend la détection plus précoce |
| `camera_timeout` | Âge maximal d’une image | Accepte des images plus anciennes mais moins synchronisées |
| `lidar_weight` | Poids de la preuve géométrique | Rend la fusion plus conservatrice et robuste au mauvais éclairage |
| `camera_weight` | Poids de la preuve visuelle | Augmente l’influence de la caméra |
| `min_camera_signal` | Signal minimal de la caméra | Rejette les images trop sombres ou peu informatives |
| `camera_score_alpha` | Lissage temporel du score caméra | Réagit plus vite mais devient plus sensible au bruit |
| `image_stride` | Sous-échantillonnage des pixels | Augmente la vitesse CPU au prix de détails visuels |
| `detection_confidence_threshold` | Seuil de sortie fusionnée | Réduit les faux positifs lorsqu’il est augmenté |

Pour une usine sombre, commencez avec `lidar_weight: 0.75`, `camera_weight: 0.25`, `camera_score_alpha: 0.15` et `detection_confidence_threshold: 0.65`. Pour une scène bien éclairée avec des obstacles visuellement contrastés, testez `lidar_weight: 0.60`, `camera_weight: 0.40` et `camera_score_alpha: 0.30`. Modifiez un seul groupe de paramètres à la fois et comparez le taux de détection, le taux de faux positifs et la latence.

## 29. Évaluation de la politique RL de saut dans Gazebo

Le script `evaluate_jump_policy.py` exécute une campagne d’épisodes ROS 2. Il appelle `/dog/jump_python`, observe `/odom`, `/lidar/front_obstacle_distance`, `/joint_states` et `/cmd_vel`, puis calcule la réussite, la hauteur maximale, la garde au-dessus de l’obstacle, la progression avant, l’erreur latérale, une approximation de l’énergie et les chutes.

Après compilation :

```bash
ros2 run dog_factory_control evaluate_jump_policy \
  --episodes 50 \
  --duration 3.0 \
  --success-height 0.20 \
  --max-lateral-error 1.0 \
  --reset-between-episodes \
  --output results/jump_policy_eval
```

Le script produit `results/jump_policy_eval.csv` pour l’analyse tabulaire et `results/jump_policy_eval.json` pour l’archivage du résumé. Une campagne typique doit être répétée avec plusieurs seeds et plusieurs hauteurs d’obstacle. Le résultat principal est le `success_rate`, mais une politique acceptable doit aussi limiter l’énergie, les chutes et l’erreur latérale.

Le script évalue la machine à états ou toute politique qui répond au service `/dog/jump_python`. Pour évaluer une véritable politique RL, remplacez l’appel de service par une publication d’action issue de votre modèle PPO et conservez les mêmes métriques d’épisode.
