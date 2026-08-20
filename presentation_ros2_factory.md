# Présentation — Robot-chien ROS 2 dans une usine

## Diapositive 1 — Vision du projet

**Robot-chien autonome dans une usine industrielle**

Simulation ROS 2 Humble + Gazebo Classic avec perception LiDAR-caméra, navigation Nav2, évitement d’obstacles, sauts et évaluation de politique.

## Diapositive 2 — Architecture globale

```text
Gazebo Factory
  ├── Robot URDF/Xacro
  ├── LiDAR 2D /scan
  ├── Caméra /image_raw
  └── Odométrie /odom
           │
           ▼
Perception C++
  ├── Segmentation LiDAR
  ├── Fusion LiDAR-caméra
  └── Markers RViz / obstacles
           │
           ├── Autonomie /cmd_vel
           ├── Nav2 /navigate_to_pose
           └── Jump State Machine /dog/jump_python
```

## Diapositive 3 — Séparation robot et usine

**`dog_robot_description`** contient la géométrie, les articulations, le lidar, la caméra et les plugins capteurs.

**`dog_factory_environment`** contient le sol, la rampe, les barrières, les caisses et les piliers dans un monde SDF indépendant.

Cette séparation facilite la réutilisation du robot dans d’autres environnements.

## Diapositive 4 — Perception LiDAR

Le LiDAR 2D produit 360 mesures jusqu’à 12 mètres sur `/scan`.

Le nœud `lidar_obstacle_detector` filtre les mesures invalides, convertit les rayons en points cartésiens, segmente les groupes de points et publie :

- `/lidar/obstacles`
- `/lidar/front_obstacle_distance`
- `/lidar/obstacle_markers`

## Diapositive 5 — Fusion LiDAR-caméra

Le nœud `sensor_fusion_node` combine une preuve géométrique fiable et un score visuel temporellement lissé.

Les hyperparamètres principaux sont `lidar_weight`, `camera_weight`, `camera_timeout`, `camera_score_alpha`, `image_stride` et `detection_confidence_threshold`.

Sorties : `/perception/fused_obstacles` et `/perception/fusion_confidence`.

## Diapositive 6 — Navigation Nav2

Nav2 fournit :

- AMCL et la carte d’occupation.
- NavFn pour le chemin global.
- DWB pour la commande locale.
- Costmaps globales et locales alimentées par `/scan`.
- Behavior Tree Navigator et comportements de récupération.

Le lancement complet est :

```bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=true navigation:=true
```

## Diapositive 7 — Sauts par-dessus les obstacles

La machine à états Python utilise les états `IDLE`, `CROUCH`, `TAKEOFF`, `FLIGHT` et `LAND`.

Le service `/dog/jump_python` déclenche la séquence. La distance frontale LiDAR peut activer un saut automatique.

Une politique RL PPO peut remplacer la machine à états après validation dans Gazebo.

## Diapositive 8 — Configuration RL

Les hyperparamètres sont dans `config/rl_jump.yaml`.

Valeurs de départ : learning rate `3e-4`, gamma `0.99`, GAE lambda `0.95`, clipping `0.2`, 16 environnements, bruit d’action décroissant de `0.20` à `0.03`.

Le curriculum augmente progressivement la hauteur des obstacles de `0.15 m` à `0.60 m`.

## Diapositive 9 — Évaluation expérimentale

`evaluate_jump_policy` exécute des épisodes et enregistre :

- taux de réussite ;
- garde verticale ;
- progression avant ;
- erreur latérale ;
- distance frontale minimale ;
- proxy d’énergie ;
- chutes.

Les résultats sont exportés en CSV et JSON pour comparer les politiques et les seeds.

## Diapositive 10 — Démonstration locale et prochaines étapes

Sur Ubuntu 22.04 + ROS 2 Humble : installer Gazebo, Nav2 et RViz2, compiler avec `colcon build`, puis lancer le bringup complet.

Prochaines étapes : ros2_control, contrôleur quadrupède dynamique, calibration caméra-LiDAR, SLAM Toolbox, politique PPO entraînée et tests sur obstacles inconnus.

Références : [ROS 2 Humble](https://docs.ros.org/en/humble/), [Nav2](https://docs.nav2.org/), [Gazebo Classic](https://classic.gazebosim.org/tutorials).
