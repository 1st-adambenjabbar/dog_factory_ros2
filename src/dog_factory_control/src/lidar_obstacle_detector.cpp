// Nœud C++ de perception LiDAR pour le robot-chien de l'usine.
// Le nœud reçoit un LaserScan, segmente les points voisins et publie des obstacles.

// Inclut l'API ROS 2 C++.
#include <rclcpp/rclcpp.hpp>

// Inclut le message du lidar 2D.
#include <sensor_msgs/msg/laser_scan.hpp>

// Inclut une position 2D exprimée sous forme de PoseArray.
#include <geometry_msgs/msg/pose_array.hpp>

// Inclut un nombre flottant contenant la distance au danger frontal.
#include <std_msgs/msg/float32.hpp>

// Inclut les marqueurs affichables dans RViz2.
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Inclut les outils mathématiques et conteneurs standard.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// Regroupe une mesure valide et ses coordonnées cartésiennes.
struct DetectionPoint
{
  // Angle polaire de la mesure en radians.
  double angle;

  // Distance mesurée par le rayon lidar.
  double range;

  // Coordonnée x dans le frame du lidar.
  double x;

  // Coordonnée y dans le frame du lidar.
  double y;
};

// Regroupe un obstacle segmenté.
struct ObstacleCluster
{
  // Liste des points appartenant au même obstacle.
  std::vector<DetectionPoint> points;
};

// Nœud ROS 2 qui détecte des groupes d'obstacles à partir du LaserScan.
class LidarObstacleDetector : public rclcpp::Node
{
public:
  // Initialise les paramètres et les interfaces ROS 2.
  LidarObstacleDetector()
  : Node("lidar_obstacle_detector")
  {
    // Distance minimale acceptée pour filtrer le bruit proche du capteur.
    declare_parameter<double>("min_range", 0.12);

    // Distance maximale utilisée par le détecteur.
    declare_parameter<double>("max_range", 12.0);

    // Écart maximal entre deux points pour rester dans un même cluster.
    declare_parameter<double>("cluster_distance", 0.35);

    // Taille minimale d'un cluster pour devenir un obstacle publié.
    declare_parameter<int>("min_cluster_points", 3);

    // Distance devant le robot considérée comme dangereuse.
    declare_parameter<double>("front_danger_distance", 1.0);

    // Reçoit les scans produits par le lidar Gazebo.
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&LidarObstacleDetector::scan_callback, this, std::placeholders::_1));

    // Publie les centres des obstacles dans le frame du lidar.
    obstacle_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/lidar/obstacles", 10);

    // Publie la distance du danger frontal pour les autres contrôleurs.
    front_distance_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/lidar/front_obstacle_distance", 10);

    // Publie des cylindres de visualisation pour RViz2.
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/lidar/obstacle_markers", 10);

    // Signale que le nœud est prêt à traiter les scans.
    RCLCPP_INFO(get_logger(), "Lidar obstacle detector ready on /scan");
  }

private:
  // Reçoit un scan et déclenche toute la chaîne de perception.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // Charge les paramètres courants afin de permettre leur modification à chaud.
    const double min_range = get_parameter("min_range").as_double();
    const double max_range = get_parameter("max_range").as_double();
    const double cluster_distance = get_parameter("cluster_distance").as_double();
    const int min_cluster_points = get_parameter("min_cluster_points").as_int();
    const double front_danger_distance = get_parameter("front_danger_distance").as_double();

    // Transforme les rayons valides en points cartésiens.
    const auto points = filter_scan(*scan, min_range, max_range);

    // Regroupe les points voisins en obstacles distincts.
    const auto clusters = segment_points(points, cluster_distance, min_cluster_points);

    // Publie les centres des clusters détectés.
    publish_obstacle_poses(*scan, clusters);

    // Publie des objets graphiques dans RViz2.
    publish_markers(*scan, clusters);

    // Publie la distance la plus proche dans le cône avant.
    publish_front_distance(points, front_danger_distance);
  }

  // Filtre les mesures invalides et calcule leurs coordonnées x/y.
  std::vector<DetectionPoint> filter_scan(
    const sensor_msgs::msg::LaserScan & scan,
    double min_range,
    double max_range)
  {
    // Préalloue un conteneur pour réduire les allocations pendant la perception.
    std::vector<DetectionPoint> points;
    points.reserve(scan.ranges.size());

    // Parcourt chaque rayon du message LaserScan.
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      // Lit la distance brute du rayon courant.
      const double range = scan.ranges[index];

      // Ignore NaN, infini et valeurs hors de la plage utile.
      if (!std::isfinite(range) || range < min_range || range > max_range) {
        continue;
      }

      // Calcule l'angle correspondant à l'indice du rayon.
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;

      // Convertit la mesure polaire en coordonnées du frame lidar.
      DetectionPoint point;
      point.angle = angle;
      point.range = range;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);

      // Ajoute le point nettoyé à la liste de perception.
      points.push_back(point);
    }

    // Retourne les points valides au segmentateur.
    return points;
  }

  // Segmente les points dans l'ordre angulaire naturel du LaserScan.
  std::vector<ObstacleCluster> segment_points(
    const std::vector<DetectionPoint> & points,
    double cluster_distance,
    int min_cluster_points)
  {
    // Prépare la sortie contenant les obstacles détectés.
    std::vector<ObstacleCluster> clusters;

    // Termine immédiatement si aucune mesure valide n'est disponible.
    if (points.empty()) {
      return clusters;
    }

    // Commence un cluster avec le premier point valide.
    ObstacleCluster current;
    current.points.push_back(points.front());

    // Compare chaque point au point précédent.
    for (std::size_t index = 1; index < points.size(); ++index) {
      // Récupère le point précédent dans le scan.
      const auto & previous = points[index - 1];

      // Récupère le point actuel dans le scan.
      const auto & current_point = points[index];

      // Mesure l'écart cartésien entre deux impacts successifs.
      const double dx = current_point.x - previous.x;
      const double dy = current_point.y - previous.y;
      const double distance = std::sqrt(dx * dx + dy * dy);

      // Conserve le point si l'écart correspond au même objet.
      if (distance <= cluster_distance) {
        current.points.push_back(current_point);
        continue;
      }

      // Publie le cluster précédent seulement s'il contient assez de points.
      if (static_cast<int>(current.points.size()) >= min_cluster_points) {
        clusters.push_back(current);
      }

      // Démarre un nouveau cluster avec le point discontinu.
      current = ObstacleCluster();
      current.points.push_back(current_point);
    }

    // N'oublie pas de traiter le dernier cluster de la boucle.
    if (static_cast<int>(current.points.size()) >= min_cluster_points) {
      clusters.push_back(current);
    }

    // Retourne les obstacles segmentés.
    return clusters;
  }

  // Publie un PoseArray contenant le centre géométrique de chaque obstacle.
  void publish_obstacle_poses(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ObstacleCluster> & clusters)
  {
    // Crée le message de positions des obstacles.
    geometry_msgs::msg::PoseArray poses;

    // Conserve le frame et l'horodatage du scan source.
    poses.header = scan.header;

    // Parcourt les clusters à publier.
    for (const auto & cluster : clusters) {
      // Ignore un cluster théoriquement vide.
      if (cluster.points.empty()) {
        continue;
      }

      // Accumule les coordonnées des points du cluster.
      double sum_x = 0.0;
      double sum_y = 0.0;

      // Additionne chaque point de l'obstacle.
      for (const auto & point : cluster.points) {
        sum_x += point.x;
        sum_y += point.y;
      }

      // Calcule le centre moyen du cluster.
      geometry_msgs::msg::Pose pose;
      pose.position.x = sum_x / static_cast<double>(cluster.points.size());
      pose.position.y = sum_y / static_cast<double>(cluster.points.size());
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;

      // Ajoute le centre à la liste publiée.
      poses.poses.push_back(pose);
    }

    // Publie toutes les poses en un seul message.
    obstacle_pose_publisher_->publish(poses);
  }

  // Publie la distance minimale dans le secteur frontal du robot.
  void publish_front_distance(
    const std::vector<DetectionPoint> & points,
    double danger_distance)
  {
    // Commence avec une distance correspondant à l'absence de détection.
    double closest = std::numeric_limits<double>::infinity();

    // Parcourt les points filtrés.
    for (const auto & point : points) {
      // Ignore les points situés hors du cône frontal de 45 degrés.
      if (std::abs(point.angle) > 0.45) {
        continue;
      }

      // Conserve la distance la plus proche dans le cône.
      closest = std::min(closest, point.range);
    }

    // Crée le message de distance frontale.
    std_msgs::msg::Float32 message;

    // Convertit l'absence de mesure en valeur max_range conventionnelle.
    message.data = std::isfinite(closest) ? static_cast<float>(closest) : 12.0F;

    // Publie la distance utilisable par l'autonomie ou un superviseur.
    front_distance_publisher_->publish(message);

    // Avertit dans les logs uniquement lorsque la zone avant est dangereuse.
    if (message.data < danger_distance) {
      RCLCPP_DEBUG(get_logger(), "Front obstacle at %.2f m", message.data);
    }
  }

  // Publie des cylindres représentant les centres des obstacles.
  void publish_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ObstacleCluster> & clusters)
  {
    // Crée la collection de marqueurs du cycle courant.
    visualization_msgs::msg::MarkerArray markers;

    // Crée un marqueur supprimant les anciens objets devenus obsolètes.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = scan.header;
    clear_marker.ns = "lidar_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    // Commence les identifiants après le marqueur de nettoyage.
    int marker_id = 1;

    // Parcourt chaque cluster détecté.
    for (const auto & cluster : clusters) {
      // Ignore les clusters vides par sécurité.
      if (cluster.points.empty()) {
        continue;
      }

      // Calcule le centre moyen du cluster courant.
      double center_x = 0.0;
      double center_y = 0.0;
      for (const auto & point : cluster.points) {
        center_x += point.x;
        center_y += point.y;
      }
      center_x /= static_cast<double>(cluster.points.size());
      center_y /= static_cast<double>(cluster.points.size());

      // Crée un cylindre de visualisation pour l'obstacle.
      visualization_msgs::msg::Marker marker;
      marker.header = scan.header;
      marker.ns = "lidar_obstacles";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center_x;
      marker.pose.position.y = center_y;
      marker.pose.position.z = 0.35;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.35;
      marker.scale.y = 0.35;
      marker.scale.z = 0.7;
      marker.color.r = 1.0F;
      marker.color.g = 0.15F;
      marker.color.b = 0.05F;
      marker.color.a = 0.85F;
      marker.lifetime = rclcpp::Duration::from_seconds(0.25);

      // Ajoute le marqueur à la publication groupée.
      markers.markers.push_back(marker);
    }

    // Publie les marqueurs dans RViz2.
    marker_publisher_->publish(markers);
  }

  // Abonnement au topic LaserScan du lidar.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  // Publication des centres d'obstacles.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_publisher_;

  // Publication de la distance frontale minimale.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_distance_publisher_;

  // Publication des marqueurs RViz2.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

// Point d'entrée du processus ROS 2.
int main(int argc, char ** argv)
{
  // Initialise la communication ROS 2.
  rclcpp::init(argc, argv);

  // Exécute le détecteur jusqu'à Ctrl+C.
  rclcpp::spin(std::make_shared<LidarObstacleDetector>());

  // Ferme proprement le contexte ROS 2.
  rclcpp::shutdown();

  // Signale une terminaison normale au système.
  return 0;
}
