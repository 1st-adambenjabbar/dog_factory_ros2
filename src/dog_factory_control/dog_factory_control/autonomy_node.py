#!/usr/bin/env python3
"""Nœud d'autonomie simple pour le robot-chien de l'usine."""

# Importe math pour tester les valeurs infinies et manipuler les angles.
import math

# Importe rclpy, la bibliothèque Python officielle de ROS 2.
import rclpy

# Importe la classe de base d'un nœud ROS 2 Python.
from rclpy.node import Node

# Importe le message LaserScan publié par le lidar.
from sensor_msgs.msg import LaserScan

# Importe le message Twist utilisé pour commander une vitesse.
from geometry_msgs.msg import Twist

# Importe le service utilisé pour réactiver l'autonomie.
from std_srvs.srv import Trigger


class FactoryDogAutonomy(Node):
    """Contrôleur réactif basé sur le lidar 2D."""

    def __init__(self):
        # Initialise le nœud avec un nom visible dans `ros2 node list`.
        super().__init__('factory_dog_autonomy')

        # Déclare la vitesse normale d'avance en mètres par seconde.
        self.declare_parameter('cruise_speed', 0.65)

        # Déclare la distance minimale avant de considérer un obstacle dangereux.
        self.declare_parameter('safe_distance', 1.0)

        # Stocke le dernier message lidar reçu.
        self.scan = None

        # Stocke le mode logique courant pour faciliter le diagnostic.
        self.mode = 'CRUISE'

        # Crée l'abonnement au topic LaserScan du capteur lidar.
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Crée l'éditeur des commandes de vitesse du robot.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Exécute la décision de conduite toutes les 100 millisecondes.
        self.create_timer(0.1, self.control_loop)

        # Offre un service permettant de réactiver le mode autonome.
        self.create_service(Trigger, '/dog/enable_autonomy', self.enable)

        # Informe l'utilisateur que le nœud est prêt.
        self.get_logger().info('Autonomie active : lidar /scan, évitement et vitesse /cmd_vel')

    def scan_cb(self, msg):
        """Mémorise le dernier scan envoyé par Gazebo."""

        # Remplace l'ancien scan par le message le plus récent.
        self.scan = msg

    def enable(self, request, response):
        """Réactive le mode de conduite autonome via un service ROS 2."""

        # Le paramètre request est conservé pour respecter la signature du service.
        del request

        # Replace le comportement dans son état normal de croisière.
        self.mode = 'CRUISE'

        # Indique au client que la requête a réussi.
        response.success = True

        # Ajoute un texte lisible dans la réponse du service.
        response.message = 'Autonomous mode enabled'

        # Retourne la réponse au client ROS 2.
        return response

    def sector(self, start_angle, end_angle):
        """Retourne la distance minimale dans un secteur angulaire du lidar."""

        # Tant qu'aucun scan n'est reçu, on considère qu'il n'y a pas d'obstacle connu.
        if self.scan is None:
            return 99.0

        # Prépare la liste des distances valides du secteur demandé.
        values = []

        # Parcourt toutes les mesures du LaserScan.
        for index, distance in enumerate(self.scan.ranges):
            # Convertit l'indice de mesure en angle selon les métadonnées du message.
            angle = self.scan.angle_min + index * self.scan.angle_increment

            # Conserve uniquement les angles du secteur et les distances finies.
            if start_angle <= angle <= end_angle and math.isfinite(distance):
                values.append(distance)

        # Si le secteur est vide, retourne une grande distance par défaut.
        return min(values) if values else 99.0

    def control_loop(self):
        """Calcule et publie une commande de vitesse à partir du lidar."""

        # Crée une commande initialement nulle pour garantir un arrêt sûr.
        command = Twist()

        # Mesure les obstacles dans le secteur avant du lidar.
        front = self.sector(-0.45, 0.45)

        # Mesure les obstacles du côté gauche du lidar.
        left = self.sector(0.45, 1.7)

        # Mesure les obstacles du côté droit du lidar.
        right = self.sector(-1.7, -0.45)

        # Lit la distance de sécurité depuis les paramètres ROS 2.
        safe_distance = self.get_parameter('safe_distance').value

        # Si un obstacle est trop proche, passe en mode évitement.
        if front < safe_distance:
            # Met à jour le mode pour faciliter l'observation dans les logs.
            self.mode = 'AVOID'

            # Interdit l'avance pendant la rotation d'évitement.
            command.linear.x = 0.0

            # Tourne vers le côté qui offre la plus grande distance libre.
            command.angular.z = 0.9 if left > right else -0.9
        else:
            # Aucun obstacle frontal critique : retourne en mode croisière.
            self.mode = 'CRUISE'

            # Avance à la vitesse paramétrée.
            command.linear.x = float(self.get_parameter('cruise_speed').value)

            # Applique une petite correction latérale si les deux côtés sont comparables.
            command.angular.z = 0.18 * (left - right) if abs(left - right) < 5.0 else 0.0

        # Publie la commande afin que le système de simulation puisse l'utiliser.
        self.cmd_pub.publish(command)


def main(args=None):
    """Point d'entrée appelé par `ros2 run`."""

    # Initialise le contexte ROS 2 avec les arguments de la ligne de commande.
    rclpy.init(args=args)

    # Construit le nœud d'autonomie.
    node = FactoryDogAutonomy()

    # Maintient le nœud actif jusqu'à Ctrl+C.
    rclpy.spin(node)

    # Libère explicitement les ressources du nœud.
    node.destroy_node()

    # Ferme proprement le contexte ROS 2.
    rclpy.shutdown()


# Exécute main uniquement lorsque le fichier est lancé directement.
if __name__ == '__main__':
    main()
