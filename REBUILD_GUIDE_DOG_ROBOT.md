# Guide de reconstruction — robot-chien ROS 2 et usine

## Objectif et méthode

Ce document explique comment reconstruire le workspace `dog_factory_ws` sans copier aveuglément les fichiers existants. L’ordre recommandé est important : commencer par une géométrie minimale, vérifier les repères et la physique, ajouter les capteurs, puis seulement ajouter la perception, le saut, Nav2 et l’apprentissage.

> Principe d’ingénierie : valider une couche avant de dépendre d’elle dans la couche suivante.

## 1. Préparer le workspace

```bash
mkdir -p ~/dog_factory_ws/src
cd ~/dog_factory_ws
source /opt/ros/humble/setup.bash
```

Créez d’abord les packages `dog_robot_description`, `dog_factory_environment`, `dog_factory_control`, `dog_factory_bringup` et `dog_factory_navigation`. Le `package.xml` déclare les dépendances ROS 2 ; le `CMakeLists.txt` installe les dossiers ; le `setup.py` expose les exécutables Python.

## 2. Comprendre la mécanique du quadrupède

Le robot-chien est un système multibody. Le châssis est le corps principal. Chaque jambe contient typiquement une hanche, un haut de jambe et un bas de jambe. Dans cette base pédagogique, les jambes et les commandes sont simplifiées ; une locomotion quadrupède réaliste doit ajouter quatre chaînes cinématiques complètes, des moteurs et des contrôleurs d’articulations.

Pour une jambe plane à deux articulations, avec longueurs `L1` et `L2` et angles `q1`, `q2`, la position du pied dans le plan est :

```text
x = L1 cos(q1) + L2 cos(q1 + q2)
z = L1 sin(q1) + L2 sin(q1 + q2)
```

La cinématique inverse consiste à retrouver les angles depuis `(x,z)`. Une solution classique est :

```text
c2 = (x² + z² - L1² - L2²) / (2 L1 L2)
q2 = atan2(±sqrt(1-c2²), c2)
q1 = atan2(z,x) - atan2(L2 sin(q2), L1 + L2 cos(q2))
```

Le code de génération de robot traduit ces relations indirectement : les valeurs `origin`, `axis` et `limit` des joints Xacro définissent la chaîne utilisée ensuite par TF, MoveIt ou un contrôleur locomotion.

La dynamique générale suit :

```text
M(q) q¨ + C(q,q˙) q˙ + g(q) + τ_friction = τ_moteur + J(q)^T F_contact
```

`M` représente l’inertie, `C` les effets de Coriolis, `g` la gravité, `τ_friction` les pertes, et `J^T F_contact` la réaction du sol. Gazebo calcule numériquement ces termes à partir des masses, inerties, collisions, joints, friction et gravité du modèle.

## 3. Construire la description robot

Commencez par `dog_robot_core.xacro`, puis séparez les composants dans `dog_robot.urdf.xacro`. Pour chaque lien, définissez trois choses : une géométrie `visual`, une géométrie `collision` et une masse `inertial`. La géométrie visuelle sert à l’affichage ; la collision sert aux contacts ; l’inertie sert à la dynamique.

Chaque joint doit préciser `parent`, `child`, `origin`, `axis`, limites angulaires et effort. Un mauvais signe dans `axis` inverse le mouvement. Un mauvais `origin` crée une discontinuité TF. Après chaque modification, utilisez :

```bash
ros2 run xacro xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro > /tmp/dog.urdf
check_urdf /tmp/dog.urdf
```

Les fichiers `urdf/dog_robot.urdf.xacro` et `urdf/dog_robot_core.xacro` traduisent donc le modèle mécanique en arbre de liens ROS 2.

## 4. Construire l’usine Gazebo

Le monde `dog_factory_environment/worlds/factory.world` définit le sol, les murs, les caisses, les piliers et les obstacles. Le robot et l’usine restent séparés : une modification du rayonnage ne doit pas modifier l’URDF.

Dans SDF, les éléments `collision`, `surface`, `friction`, `contact`, `mass` et `inertia` influencent la simulation. La force de contact approximative dépend de la pénétration et du modèle de rigidité ; des valeurs trop rigides rendent le moteur instable, tandis que des valeurs trop faibles font pénétrer les objets.

## 5. Comprendre les signaux ROS 2

ROS 2 transforme la mécanique en flux de messages. Le robot publie des états ; les capteurs publient des mesures ; les contrôleurs publient des commandes.

| Signal | Type | Sens physique |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Distances radiales du lidar |
| `/odom` | `nav_msgs/Odometry` | Pose et vitesse estimées |
| `/cmd_vel` | `geometry_msgs/Twist` | Vitesse linéaire et angulaire demandée |
| `/joint_states` | `sensor_msgs/JointState` | Angles, vitesses et efforts articulaires |
| `/camera/image_raw` | `sensor_msgs/Image` | Intensités des pixels caméra |
| `/lidar/obstacles` | `geometry_msgs/PoseArray` | Centres d’obstacles segmentés |

Une mesure lidar se convertit en point avec :

```text
x_i = r_i cos(θ_i)
y_i = r_i sin(θ_i)
```

C’est exactement le principe utilisé par `lidar_obstacle_detector.cpp`. Les rayons invalides sont rejetés, les points proches sont regroupés, puis les groupes sont publiés en poses.

## 6. Évitement, fusion et navigation

`autonomy_node.py` observe `/scan` et calcule une commande réactive. Une logique simple compare les distances gauche, centre et droite. Une commande de rotation est produite quand la distance frontale est inférieure au seuil.

`sensor_fusion_node.cpp` combine une preuve lidar et un score caméra. Une fusion linéaire typique est :

```text
score = w_lidar score_lidar + w_camera score_camera
```

avec `w_lidar + w_camera = 1`. Les paramètres se trouvent dans `sensor_fusion.yaml`. Une mesure caméra trop ancienne doit être rejetée pour éviter de fusionner des instants différents.

Nav2 ajoute une carte, une localisation AMCL, un planificateur global et un contrôleur local. Le lidar construit ou met à jour la costmap ; Nav2 transforme l’objectif en chemin ; le contrôleur publie `/cmd_vel`.

## 7. Machine de saut

`jump_state_machine.py` sépare les états `IDLE`, `CROUCH`, `TAKEOFF`, `FLIGHT` et `LAND`. Le saut est un profil temporel de positions articulaires. Pour une trajectoire verticale de référence, on peut utiliser :

```text
z(t) = z0 + v0 t - 1/2 g t²
```

Le temps de vol idéal sans résistance est `T = 2 v0 / g`, et la hauteur maximale supplémentaire est `Δh = v0² / (2g)`. En simulation, les articulations doivent produire cette accélération via les contacts au sol ; il faut donc limiter l’effort et valider la réception.

Le fichier `rl_jump.yaml` contient les hyperparamètres PPO, les récompenses et le curriculum, mais la machine à états reste la référence déterministe immédiatement exécutable.

## 8. Ordre exact pour recoder

Commencez par afficher uniquement un châssis. Ajoutez les jambes et vérifiez les axes. Ajoutez ensuite masses et collisions. Démarrez Gazebo et observez si le robot tombe correctement sous la gravité. Ajoutez les transmissions et contrôleurs. Ajoutez le lidar et vérifiez `/scan`. Ajoutez l’odom et `/cmd_vel`. Ajoutez ensuite le détecteur C++, la fusion caméra et Nav2. Enfin, implémentez la machine de saut, l’évaluateur et RL.

Ne commencez pas par Nav2 ou RL : sans TF, odométrie, collision et contrôle articulé fiables, les couches supérieures ne peuvent pas être diagnostiquées.

## 9. Correspondance code → concept

| Concept | Fichier à lire en premier | Ce que le code traduit |
|---|---|---|
| Arbre mécanique | `dog_robot.urdf.xacro` | Liens, joints, repères |
| Physique usine | `factory.world` | Sol, obstacles, contacts |
| Lecture lidar | `lidar_obstacle_detector.cpp` | `r,θ → x,y`, segmentation |
| Fusion | `sensor_fusion_node.cpp` | Combinaison de scores |
| Saut | `jump_state_machine.py` | États et profil temporel |
| Navigation | `navigation.launch.py`, `nav2_params.yaml` | Carte, costmaps, planification |
| Vérification | `test_lidar_obstacle_detector.py` | Scénarios synthétiques |

## 10. Limites pédagogiques

Le workspace est une architecture d’apprentissage et de simulation. Un vrai quadrupède exige une dynamique de contact calibrée, des moteurs, des contrôleurs d’effort, une estimation d’état robuste, une synchronisation capteurs et des essais de sécurité. Les équations expliquent les principes ; elles ne constituent pas à elles seules une commande stable du robot réel.

## Références

[1]: https://docs.ros.org/en/humble/ — Documentation ROS 2 Humble.
[2]: https://classic.gazebosim.org/tutorials — Tutoriels Gazebo Classic.
[3]: https://docs.nav2.org/ — Documentation Nav2.
