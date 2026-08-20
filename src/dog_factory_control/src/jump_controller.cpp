// Inclut la classe Node nécessaire pour créer un nœud ROS 2 en C++.
#include <rclcpp/rclcpp.hpp>

// Inclut le message contenant une liste de joints et des points temporels.
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Inclut le type de service Trigger utilisé pour demander un saut.
#include <std_srvs/srv/trigger.hpp>

// Inclut les outils standard pour les pointeurs partagés et les vecteurs.
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Utilise les littéraux de durée comme 1600ms.
using namespace std::chrono_literals;

// Déclare le nœud qui génère les positions du saut.
class JumpController : public rclcpp::Node
{
public:
  // Construit le nœud et crée les interfaces ROS 2.
  JumpController()
  : Node("jump_controller_cpp"), jump_running_(false)
  {
    // Publie les trajectoires destinées aux articulations du robot.
    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/dog/joint_trajectory", 10);

    // Crée le service qui déclenche une séquence de saut.
    jump_service_ = create_service<std_srvs::srv::Trigger>(
      "/dog/jump",
      std::bind(
        &JumpController::handle_jump,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Informe l'utilisateur que le service est prêt.
    RCLCPP_INFO(get_logger(), "Jump controller ready on /dog/jump");
  }

private:
  // Répond à une requête de saut et publie trois positions articulaires.
  void handle_jump(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Le service Trigger ne contient pas de champ utile dans la requête.
    (void)request;

    // Refuse une deuxième séquence tant que la première n'est pas terminée.
    if (jump_running_) {
      response->success = false;
      response->message = "Jump already running";
      return;
    }

    // Marque le contrôleur comme occupé.
    jump_running_ = true;

    // Crée le message de trajectoire à publier.
    trajectory_msgs::msg::JointTrajectory trajectory;

    // Décrit l'ordre exact des articulations dans chaque point.
    trajectory.joint_names = {
      "front_left_knee_joint", "front_right_knee_joint",
      "rear_left_knee_joint", "rear_right_knee_joint",
      "front_left_ankle_joint", "front_right_ankle_joint",
      "rear_left_ankle_joint", "rear_right_ankle_joint"};

    // Position accroupie avant l'impulsion.
    const std::vector<double> crouch = {
      -1.8, -1.8, -1.8, -1.8, 0.9, 0.9, 0.9, 0.9};

    // Position étendue utilisée pendant le saut.
    const std::vector<double> extension = {
      -0.7, -0.7, -0.7, -0.7, 0.2, 0.2, 0.2, 0.2};

    // Position de réception après le retour vers le sol.
    const std::vector<double> landing = {
      -1.2, -1.2, -1.2, -1.2, 0.5, 0.5, 0.5, 0.5};

    // Ajoute le premier point avec un temps relatif de 0.4 seconde.
    add_point(trajectory, crouch, 0.4);

    // Ajoute le deuxième point d'extension à 0.8 seconde.
    add_point(trajectory, extension, 0.8);

    // Ajoute le point final de réception à 1.4 seconde.
    add_point(trajectory, landing, 1.4);

    // Publie la trajectoire complète sur le topic dédié.
    trajectory_publisher_->publish(trajectory);

    // Crée un minuteur qui libère le contrôleur après la séquence.
    reset_timer_ = create_wall_timer(
      1600ms,
      [this]() {
        // Rend possible un nouveau saut.
        jump_running_ = false;

        // Annule ce minuteur à usage unique.
        reset_timer_->cancel();
      });

    // Confirme au client que la trajectoire est partie.
    response->success = true;
    response->message = "Jump trajectory published";
  }

  // Ajoute un point temporellement ordonné à une trajectoire.
  void add_point(
    trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<double> & positions,
    double time_seconds)
  {
    // Construit un nouveau point de trajectoire.
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Copie les positions articulaires du point courant.
    point.positions = positions;

    // Convertit le temps en durée ROS 2.
    point.time_from_start = rclcpp::Duration::from_seconds(time_seconds);

    // Ajoute le point à la liste publiée.
    trajectory.points.push_back(point);
  }

  // Publie les trajectoires de saut.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  // Serveur du service /dog/jump.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr jump_service_;

  // Minuteur qui indique la fin de la séquence.
  rclcpp::TimerBase::SharedPtr reset_timer_;

  // Indique si un saut est en cours.
  bool jump_running_;
};

// Fonction principale du programme C++.
int main(int argc, char ** argv)
{
  // Initialise la bibliothèque ROS 2.
  rclcpp::init(argc, argv);

  // Exécute le nœud jusqu'à son arrêt.
  rclcpp::spin(std::make_shared<JumpController>());

  // Ferme proprement ROS 2.
  rclcpp::shutdown();

  // Retourne un code système indiquant le succès.
  return 0;
}
