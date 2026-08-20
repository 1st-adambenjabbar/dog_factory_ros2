# Guide de lecture et d’annotation du code

## `dog_factory_description/urdf/dog.urdf.xacro`

Ce fichier décrit les liens, les joints, les masses, les collisions, les matériaux et les capteurs. Les sections importantes sont `base_link`, la macro `leg`, les quatre instanciations de jambes, le bloc caméra et le bloc lidar. Le lidar est déclaré comme capteur Gazebo de type `ray` et utilise `libgazebo_ros_ray_sensor.so` pour publier `/scan`.

Le fichier original est très compact afin de réduire sa taille. Pour une lecture pédagogique approfondie, utilisez les noms explicites des éléments et les explications du README. Les valeurs physiques sont exprimées en unités SI : mètres, kilogrammes, radians et secondes.

## `dog_factory_control/dog_factory_control/autonomy_node.py`

Le fichier est annoté directement. Chaque groupe d’instructions est précédé d’un commentaire en français. Les commentaires expliquent les imports, la création du nœud, les paramètres, l’abonnement `LaserScan`, le calcul des secteurs angulaires, la comparaison avec `safe_distance` et la publication de `Twist`.

La fonction `sector()` convertit chaque indice du tableau `ranges` en angle avec `angle_min + index * angle_increment`. Elle ignore les valeurs infinies, puis retourne la distance minimale restante. La fonction `control_loop()` transforme cette mesure en comportement de croisière ou d’évitement.

## `dog_factory_control/dog_factory_control/keyboard_teleop.py`

Le fichier est annoté directement. Il explique le passage du terminal en mode caractère immédiat, la lecture non bloquante avec `select`, la conversion des touches en `Twist`, la restauration du terminal et l’arrêt propre de ROS 2.

## `dog_factory_control/src/jump_controller.cpp`

Le fichier est annoté directement. Les commentaires suivent la structure C++ : includes, constructeur, éditeur de trajectoire, service `/dog/jump`, points accroupissement-extension-réception, minuteur de fin et fonction `main()`.

La méthode `add_point()` centralise la création des points de trajectoire. Cette organisation évite de recopier trois fois la même logique et facilite la modification des temps ou des positions articulaires.

## `dog_factory_bringup/launch/factory_sim.launch.py`

Le fichier est annoté directement. Chaque action est documentée : recherche des packages, construction des chemins, inclusion de `gazebo.launch.py`, conversion Xacro, insertion de l’entité, lancement de l’autonomie et lancement du contrôleur C++.

## Fichiers XML, SDF et YAML

Les fichiers déclaratifs sont naturellement moins adaptés à un commentaire sur chaque ligne, car certaines lignes contiennent plusieurs éléments XML compacts. Le README décrit toutes les sections fonctionnelles : modèle robot, capteurs, monde usine, paramètres et topics. Pour une version de production ou un cours, il est recommandé de reformater le Xacro et le monde SDF avec un élément XML par ligne avant d’ajouter des commentaires XML section par section.

## Vérification rapide

```bash
python3 -m py_compile \
  src/dog_factory_control/dog_factory_control/autonomy_node.py \
  src/dog_factory_control/dog_factory_control/keyboard_teleop.py \
  src/dog_factory_bringup/launch/factory_sim.launch.py

grep -RInE 'lidar_link|LaserScan|/scan|gazebo_ros_ray_sensor' src README.md
```
