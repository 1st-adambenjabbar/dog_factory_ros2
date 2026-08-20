#!/usr/bin/env python3
"""Contrôleur de saut Python basé sur une machine à états explicite."""

# Importe les messages et primitives de ROS 2 Python.
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# Importe les messages LiDAR, trajectoire et le service de déclenchement.
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JumpStateMachine(Node):
    """Simule une séquence accroupissement, impulsion, vol et réception."""

    def __init__(self):
        # Initialise le nœud ROS 2.
        super().__init__('jump_state_machine')

        # Déclare le déclenchement automatique facultatif.
        self.declare_parameter('auto_jump', False)

        # Déclare le seuil frontal qui peut déclencher un saut.
        self.declare_parameter('jump_trigger_distance', 0.75)

        # Mémorise l'état courant de la machine.
        self.state = 'IDLE'

        # Mémorise la dernière distance fournie par la perception LiDAR.
        self.front_distance = 12.0

        # Mémorise l'instant du changement d'état.
        self.state_started = self.get_clock().now()

        # Publie les trajectoires articulaires de saut.
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, '/dog/joint_trajectory', 10)

        # Reçoit la distance frontale issue du détecteur LiDAR.
        self.create_subscription(
            Float32, '/lidar/front_obstacle_distance', self.distance_callback, 10)

        # Permet de demander un saut manuellement.
        self.create_service(Trigger, '/dog/jump_python', self.jump_service)

        # Met à jour la machine à états à 50 Hz.
        self.create_timer(0.02, self.update_state_machine)

        # Informe l'utilisateur du démarrage du contrôleur.
        self.get_logger().info('Python jump state machine ready')

    def distance_callback(self, message):
        """Mémorise la distance frontale publiée par la perception."""

        # Stocke la mesure flottante reçue.
        self.front_distance = float(message.data)

    def jump_service(self, request, response):
        """Démarre un saut manuel si le robot est au repos."""

        # Le service Trigger ne contient aucun champ de requête utile.
        del request

        # Refuse une nouvelle commande pendant un saut existant.
        if self.state != 'IDLE':
            response.success = False
            response.message = f'Jump rejected while state={self.state}'
            return response

        # Passe immédiatement à la phase d'accroupissement.
        self.change_state('CROUCH')

        # Confirme le démarrage au client ROS 2.
        response.success = True
        response.message = 'Jump state machine started'
        return response

    def change_state(self, new_state):
        """Change d'état et mémorise l'heure de transition."""

        # Enregistre le nouvel état logique.
        self.state = new_state

        # Réinitialise le chronomètre de l'état.
        self.state_started = self.get_clock().now()

        # Trace la transition pour le diagnostic.
        self.get_logger().info(f'Jump state -> {new_state}')

        # Publie immédiatement la posture associée au nouvel état.
        self.publish_posture(new_state)

    def state_elapsed(self):
        """Retourne le temps écoulé dans l'état courant."""

        # Calcule la durée ROS 2 depuis l'entrée dans l'état.
        elapsed = self.get_clock().now() - self.state_started

        # Convertit la durée en secondes Python.
        return elapsed.nanoseconds / 1e9

    def update_state_machine(self):
        """Exécute les transitions de la machine à états."""

        # Déclenche automatiquement un saut si le paramètre est activé.
        if self.state == 'IDLE' and self.get_parameter('auto_jump').value:
            threshold = self.get_parameter('jump_trigger_distance').value
            if self.front_distance < threshold:
                self.change_state('CROUCH')
                return

        # Passe de l'accroupissement à l'impulsion après 0.35 seconde.
        if self.state == 'CROUCH' and self.state_elapsed() >= 0.35:
            self.change_state('TAKEOFF')

        # Passe de l'impulsion à la phase de vol après 0.20 seconde.
        elif self.state == 'TAKEOFF' and self.state_elapsed() >= 0.20:
            self.change_state('FLIGHT')

        # Passe du vol à la réception après 0.45 seconde.
        elif self.state == 'FLIGHT' and self.state_elapsed() >= 0.45:
            self.change_state('LAND')

        # Retourne au repos après stabilisation de la réception.
        elif self.state == 'LAND' and self.state_elapsed() >= 0.55:
            self.change_state('IDLE')

    def publish_posture(self, state):
        """Publie une posture articulaire correspondant à un état."""

        # Crée la liste fixe des huit articulations commandées.
        message = JointTrajectory()
        message.joint_names = [
            'front_left_knee_joint', 'front_right_knee_joint',
            'rear_left_knee_joint', 'rear_right_knee_joint',
            'front_left_ankle_joint', 'front_right_ankle_joint',
            'rear_left_ankle_joint', 'rear_right_ankle_joint',
        ]

        # Choisit les angles selon la phase du saut.
        postures = {
            'IDLE': [-1.2, -1.2, -1.2, -1.2, 0.5, 0.5, 0.5, 0.5],
            'CROUCH': [-1.8, -1.8, -1.8, -1.8, 0.9, 0.9, 0.9, 0.9],
            'TAKEOFF': [-0.7, -0.7, -0.7, -0.7, 0.2, 0.2, 0.2, 0.2],
            'FLIGHT': [-0.4, -0.4, -0.4, -0.4, 0.0, 0.0, 0.0, 0.0],
            'LAND': [-1.2, -1.2, -1.2, -1.2, 0.5, 0.5, 0.5, 0.5],
        }

        # Construit un point avec les angles de l'état actuel.
        point = JointTrajectoryPoint()
        point.positions = postures[state]
        point.time_from_start = Duration(seconds=0.15).to_msg()

        # Ajoute le point puis publie la trajectoire.
        message.points.append(point)
        self.trajectory_publisher.publish(message)


def main(args=None):
    """Point d'entrée ROS 2 du contrôleur Python."""

    # Initialise ROS 2.
    rclpy.init(args=args)

    # Construit et exécute la machine à états.
    node = JumpStateMachine()
    rclpy.spin(node)

    # Ferme proprement le nœud et ROS 2.
    node.destroy_node()
    rclpy.shutdown()


# Lance le programme lorsque le fichier est exécuté directement.
if __name__ == '__main__':
    main()
