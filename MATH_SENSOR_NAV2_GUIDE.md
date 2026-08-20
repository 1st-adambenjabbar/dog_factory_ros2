# Mathématiques, fusion capteurs et Nav2 — robot-chien

## 1. Perception lidar

Pour un rayon lidar `r_i` à l'angle `θ_i`, le point dans le repère capteur est :

```text
p_i = [r_i cos(θ_i), r_i sin(θ_i), 0, 1]^T
```

La transformation vers `base_link` est donnée par TF :

```text
p_base = T_base_lidar p_lidar
```

Le nœud `lidar_obstacle_detector.cpp` filtre les valeurs non finies, transforme les rayons en points et regroupe les points voisins pour estimer les obstacles.

## 2. Fusion lidar-caméra

Le score de détection est une combinaison pondérée :

```text
S = w_L S_L + w_C S_C
```

avec `w_L+w_C=1`. Une image trop ancienne est rejetée. Le nœud `sensor_fusion_node.cpp` publie une confiance fusionnée et des obstacles. Les paramètres sont dans `src/dog_factory_control/config/sensor_fusion.yaml`.

Pour tester :

```bash
source /opt/ros/humble/setup.bash
source ~/dog_factory_ws/install/setup.bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=true navigation:=false
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic echo /perception/fused_obstacles
ros2 topic echo /perception/fusion_confidence
```

Placez successivement des obstacles frontaux, latéraux, fins et partiellement masqués. Comparez `lidar_weight`, `camera_weight`, `camera_timeout` et `detection_confidence_threshold` une campagne à la fois.

## 3. Navigation Nav2

Nav2 utilise une carte, une localisation et des costmaps. Les repères doivent relier `map → odom → base_link`. Le lidar alimente la costmap locale et le contrôleur publie `/cmd_vel`.

Lancement :

```bash
source /opt/ros/humble/setup.bash
source ~/dog_factory_ws/install/setup.bash
ros2 launch dog_factory_bringup factory_sim.launch.py gui:=true navigation:=true
```

Vérifications :

```bash
ros2 node list | grep -E 'amcl|planner|controller|bt_navigator|map_server'
ros2 topic list | grep -E 'map|costmap|plan|scan|odom'
ros2 run tf2_ros tf2_echo map base_link
```

Dans RViz, choisissez `map` comme Fixed Frame, affichez la carte, TF, laser, costmaps et chemins, puis utilisez `2D Pose Estimate` avant `Nav2 Goal`. Testez une allée libre, un obstacle ajouté et un passage bloqué. Le résultat attendu est une déviation, une récupération ou un échec explicite, jamais une collision silencieuse.

## 4. Robot-chien et équations de locomotion

Pour une jambe plane à deux segments :

```text
x = L1 cos(q1) + L2 cos(q1+q2)
z = L1 sin(q1) + L2 sin(q1+q2)
```

L'IK correspondante est :

```text
c2 = (x²+z²-L1²-L2²)/(2L1L2)
q2 = atan2(±sqrt(1-c2²), c2)
q1 = atan2(z,x)-atan2(L2 sin(q2), L1+L2 cos(q2))
```

Ces relations sont traduites dans les `origin`, `axis`, limites et chaînes de liens Xacro ; un contrôleur complet doit ensuite convertir les positions de pied en commandes de joints.

## 5. Références

[1]: https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html — Topics ROS 2.
[2]: https://moveit.picknik.ai/humble/doc/concepts/kinematics.html — Cinématique MoveIt 2.
[3]: https://docs.nav2.org/getting_started/index.html — Démarrage Nav2 sous simulation.
