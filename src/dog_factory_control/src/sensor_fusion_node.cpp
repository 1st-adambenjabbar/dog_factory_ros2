// Nœud C++ de fusion simple entre LiDAR 2D et caméra RGB.
// La branche LiDAR fournit une géométrie fiable ; la branche caméra fournit un score visuel.

// API ROS 2 C++.
#include <rclcpp/rclcpp.hpp>

// Message des distances du lidar.
#include <sensor_msgs/msg/laser_scan.hpp>

// Message image brut publié par Gazebo.
#include <sensor_msgs/msg/image.hpp>

// Message d'obstacles fusionnés.
#include <geometry_msgs/msg/pose_array.hpp>

// Message de diagnostic de confiance.
#include <std_msgs/msg/float32.hpp>

// Outils numériques et mémoire.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

// Nœud fusionnant les deux capteurs du robot.
class SensorFusionNode : public rclcpp::Node
{
public:
  // Initialise les paramètres et les abonnements.
  SensorFusionNode()
  : Node("sensor_fusion_node"), latest_image_score_(0.0F), latest_image_stamp_(0, 0, RCL_ROS_TIME)
  {
    // Définit le topic image par défaut du plugin Gazebo.
    declare_parameter<std::string>("image_topic", "/image_raw");

    // Définit le secteur avant utilisé pour la fusion.
    declare_parameter<double>("front_angle", 0.45);

    // Définit le seuil de distance considéré comme obstacle.
    declare_parameter<double>("obstacle_distance", 2.0);

    // Définit la durée maximale de validité du score caméra.
    declare_parameter<double>("camera_timeout", 0.5);

    // Reçoit chaque scan lidar avec une QoS adaptée aux capteurs.
    lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&SensorFusionNode::lidar_callback, this, std::placeholders::_1));

    // Reçoit les images brutes de la caméra Gazebo.
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&SensorFusionNode::image_callback, this, std::placeholders::_1));

    // Publie les obstacles validés par les deux sources.
    fused_obstacle_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/perception/fused_obstacles", 10);

    // Publie la confiance issue de la fusion.
    confidence_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/perception/fusion_confidence", 10);

    // Informe l'utilisateur de la mise en route.
    RCLCPP_INFO(get_logger(), "Sensor fusion ready: LiDAR /scan + camera %s", get_parameter("image_topic").as_string().c_str());
  }

private:
  // Traite l'image et calcule un score de contraste central léger.
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image)
  {
    // Ignore les images vides ou sans données.
    if (image->width == 0 || image->height == 0 || image->data.empty()) {
      return;
    }

    // Calcule les limites d'une fenêtre centrale de l'image.
    const std::size_t min_x = image->width / 4;
    const std::size_t max_x = (image->width * 3) / 4;
    const std::size_t min_y = image->height / 4;
    const std::size_t max_y = (image->height * 3) / 4;

    // Accumule la luminosité normalisée de la fenêtre centrale.
    double sum = 0.0;
    std::size_t samples = 0;

    // Évite de supposer un encodage exact pour les images non supportées.
    const std::size_t channels = image->step / image->width;
    if (channels == 0) {
      return;
    }

    // Parcourt une image sous-échantillonnée pour limiter le coût CPU.
    for (std::size_t y = min_y; y < max_y; y += 8) {
      for (std::size_t x = min_x; x < max_x; x += 8) {
        // Calcule l'index du pixel dans le tableau binaire.
        const std::size_t index = y * image->step + x * channels;

        // Vérifie que l'accès reste dans la mémoire du message.
        if (index >= image->data.size()) {
          continue;
        }

        // Utilise le premier canal comme indicateur de structure visuelle.
        sum += static_cast<double>(image->data[index]) / 255.0;
        ++samples;
      }
    }

    // Stocke la moyenne centrale pour la prochaine fusion.
    latest_image_score_ = samples == 0 ? 0.0F : static_cast<float>(sum / samples);

    // Stocke l'horodatage de la dernière image disponible.
    latest_image_stamp_ = rclcpp::Time(image->header.stamp);
  }

  // Calcule la distance frontale du lidar et fusionne avec le score image.
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // Charge les paramètres de fusion courants.
    const double front_angle = get_parameter("front_angle").as_double();
    const double obstacle_distance = get_parameter("obstacle_distance").as_double();
    const double camera_timeout = get_parameter("camera_timeout").as_double();

    // Initialise la distance sans obstacle détecté.
    double closest_distance = std::numeric_limits<double>::infinity();

    // Parcourt les rayons du secteur frontal.
    for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
      // Convertit l'indice en angle.
      const double angle = scan->angle_min + static_cast<double>(index) * scan->angle_increment;

      // Ignore les rayons hors du secteur frontal.
      if (std::abs(angle) > front_angle) {
        continue;
      }

      // Ignore les valeurs invalides du LaserScan.
      const double range = scan->ranges[index];
      if (!std::isfinite(range)) {
        continue;
      }

      // Conserve la distance la plus courte.
      closest_distance = std::min(closest_distance, range);
    }

    // Vérifie si l'image est encore récente.
    const double image_age = (this->get_clock()->now() - latest_image_stamp_).seconds();
    const bool camera_available = image_age >= 0.0 && image_age <= camera_timeout;

    // Le LiDAR apporte une confiance forte dès qu'un obstacle géométrique est proche.
    const bool lidar_obstacle = std::isfinite(closest_distance) && closest_distance < obstacle_distance;

    // Le score visuel augmente la confiance si une image récente est disponible.
    const float camera_weight = camera_available ? latest_image_score_ : 0.0F;

    // Combine une preuve géométrique et une preuve visuelle sans remplacer le LiDAR.
    const float confidence = lidar_obstacle ? std::min(1.0F, 0.65F + 0.35F * camera_weight) : 0.35F * camera_weight;

    // Publie un PoseArray contenant l'obstacle frontal lorsqu'il existe.
    geometry_msgs::msg::PoseArray obstacles;
    obstacles.header = scan->header;

    // Ajoute le centre approximatif de l'obstacle dans le frame du lidar.
    if (lidar_obstacle) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = closest_distance;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      obstacles.poses.push_back(pose);
    }

    // Publie la sortie fusionnée pour un planificateur ou RViz2.
    fused_obstacle_publisher_->publish(obstacles);

    // Publie le score de confiance associé au cycle courant.
    std_msgs::msg::Float32 confidence_message;
    confidence_message.data = confidence;
    confidence_publisher_->publish(confidence_message);
  }

  // Abonnement au flux LiDAR.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

  // Abonnement au flux caméra.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publication des obstacles fusionnés.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fused_obstacle_publisher_;

  // Publication de la confiance de fusion.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_publisher_;

  // Score moyen de la fenêtre centrale de la dernière image.
  float latest_image_score_;

  // Timestamp de la dernière image valide.
  rclcpp::Time latest_image_stamp_;
};

// Point d'entrée du nœud de fusion.
int main(int argc, char ** argv)
{
  // Initialise ROS 2.
  rclcpp::init(argc, argv);

  // Exécute le nœud jusqu'à son arrêt.
  rclcpp::spin(std::make_shared<SensorFusionNode>());

  // Ferme la communication ROS 2.
  rclcpp::shutdown();

  // Retourne un code de succès.
  return 0;
}
