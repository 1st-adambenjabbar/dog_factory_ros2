#!/usr/bin/env python3
"""Téléopération clavier minimale pour le robot-chien."""

# Importe les outils Python nécessaires pour lire le clavier sans Entrée.
import select
import sys
import termios
import tty

# Importe l'API Python ROS 2.
import rclpy

# Importe la classe de base d'un nœud ROS 2.
from rclpy.node import Node

# Importe le message de vitesse utilisé sur /cmd_vel.
from geometry_msgs.msg import Twist


class Teleop(Node):
    """Publie une commande Twist à chaque touche reconnue."""

    def __init__(self):
        # Initialise le nœud avec un nom identifiable.
        super().__init__('keyboard_teleop')

        # Crée l'éditeur du topic de commande de vitesse.
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def run(self):
        """Lit les touches et publie les vitesses jusqu'à la touche q."""

        # Sauvegarde la configuration originale du terminal.
        original_settings = termios.tcgetattr(sys.stdin)

        # Passe le terminal en mode caractère immédiat.
        tty.setcbreak(sys.stdin.fileno())

        # Affiche l'aide utilisateur dans le terminal.
        print('w: avance | s: recul | a: gauche | d: droite | x: stop | q: quitter')

        try:
            # Continue tant que ROS 2 fonctionne.
            while rclpy.ok():
                # Laisse ROS traiter ses événements pendant l'attente clavier.
                rclpy.spin_once(self, timeout_sec=0.05)

                # Vérifie si un caractère est disponible sans bloquer le programme.
                readable, _, _ = select.select([sys.stdin], [], [], 0)

                # Ignore le cycle lorsqu'aucune touche n'est disponible.
                if not readable:
                    continue

                # Lit exactement un caractère au clavier.
                key = sys.stdin.read(1)

                # Crée une commande initialement nulle.
                command = Twist()

                # Associe w à une vitesse avant positive.
                if key == 'w':
                    command.linear.x = 0.8

                # Associe s à une vitesse arrière modérée.
                elif key == 's':
                    command.linear.x = -0.5

                # Associe a à une rotation gauche.
                elif key == 'a':
                    command.angular.z = 1.0

                # Associe d à une rotation droite.
                elif key == 'd':
                    command.angular.z = -1.0

                # La touche x conserve la commande nulle et arrête le robot.
                elif key == 'x':
                    pass

                # Quitte proprement la boucle avec q.
                elif key == 'q':
                    break

                # Publie la commande calculée sur /cmd_vel.
                self.publisher.publish(command)

        finally:
            # Restaure le terminal même si le programme est interrompu.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)


def main(args=None):
    """Point d'entrée utilisé par `ros2 run`."""

    # Initialise ROS 2.
    rclpy.init(args=args)

    # Construit l'objet de téléopération.
    node = Teleop()

    # Lance la boucle de lecture clavier.
    node.run()

    # Libère le nœud après la sortie.
    node.destroy_node()

    # Ferme le contexte ROS 2.
    rclpy.shutdown()


# Lance main lorsque le fichier est exécuté directement.
if __name__ == '__main__':
    main()
