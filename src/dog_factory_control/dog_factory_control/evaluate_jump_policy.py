#!/usr/bin/env python3
"""Évalue une politique de saut dans Gazebo via les topics et services ROS 2."""

# Importe les outils CLI, fichiers, statistiques et temporisation.
import argparse
import csv
import json
import math
import time
from pathlib import Path

# Importe l'API ROS 2 Python et le nœud de base.
import rclpy
from rclpy.node import Node

# Importe les messages utiles aux métriques d'évaluation.
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Empty, Trigger


class JumpPolicyEvaluator(Node):
    """Collecte les données d'un épisode et déclenche les sauts."""

    def __init__(self):
        # Initialise le nœud d'évaluation.
        super().__init__('jump_policy_evaluator')

        # Stocke les dernières mesures disponibles.
        self.odom = None
        self.front_distance = 12.0
        self.joint_state = None
        self.last_cmd = Twist()

        # Abonne le nœud aux données du robot et de la perception.
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(Float32, '/lidar/front_obstacle_distance', self.distance_callback, 20)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 20)

        # Crée les clients de déclenchement du saut et de remise à zéro Gazebo.
        self.jump_client = self.create_client(Trigger, '/dog/jump_python')
        self.reset_client = self.create_client(Empty, '/reset_world')

    def odom_callback(self, message):
        """Mémorise l'odométrie publiée par Gazebo."""
        self.odom = message

    def distance_callback(self, message):
        """Mémorise la distance frontale LiDAR."""
        self.front_distance = float(message.data)

    def joint_callback(self, message):
        """Mémorise les positions articulaires disponibles."""
        self.joint_state = message

    def cmd_callback(self, message):
        """Mémorise la dernière commande de vitesse."""
        self.last_cmd = message

    def wait_for_topics(self, timeout):
        """Attend l'arrivée de l'odométrie ou retourne False à l'expiration."""
        deadline = time.monotonic() + timeout
        while self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.odom is not None

    def reset_world(self, timeout):
        """Réinitialise Gazebo si le service /reset_world existe."""
        if not self.reset_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning('Service /reset_world absent : épisode sans reset Gazebo')
            return False
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done() and future.exception() is None

    def request_jump(self, timeout):
        """Demande un saut et retourne le résultat du service."""
        if not self.jump_client.wait_for_service(timeout_sec=timeout):
            return False, 'service_unavailable'
        future = self.jump_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            return False, 'service_timeout'
        response = future.result()
        return bool(response.success), response.message

    def run_episode(self, episode_id, duration, success_height, max_lateral_error):
        """Exécute un épisode et calcule succès, chute, énergie et progression."""
        # Capture les références initiales de position.
        start_time = time.monotonic()
        start_x = self.odom.pose.pose.position.x if self.odom else 0.0
        max_z = start_z = self.odom.pose.pose.position.z if self.odom else 0.0
        max_y_error = 0.0
        energy_proxy = 0.0
        samples = 0
        min_front_distance = self.front_distance

        # Demande le saut au contrôleur Python.
        jump_ok, jump_message = self.request_jump(timeout=3.0)

        # Observe l'épisode jusqu'à expiration de sa durée.
        while time.monotonic() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.odom is None:
                continue

            # Lit la pose courante du robot.
            pose = self.odom.pose.pose
            max_z = max(max_z, pose.position.z)
            max_y_error = max(max_y_error, abs(pose.position.y))
            min_front_distance = min(min_front_distance, self.front_distance)

            # Approxime l'énergie par la norme des commandes de vitesse.
            energy_proxy += abs(self.last_cmd.linear.x) + abs(self.last_cmd.angular.z)
            samples += 1

        # Calcule les métriques finales de l'épisode.
        final_x = self.odom.pose.pose.position.x if self.odom else start_x
        clearance = max_z - start_z
        forward_progress = final_x - start_x
        fell = self.odom is not None and self.odom.pose.pose.position.z < 0.05
        success = jump_ok and clearance >= success_height and not fell and max_y_error <= max_lateral_error

        # Retourne un dictionnaire sérialisable en CSV et JSON.
        return {
            'episode': episode_id,
            'success': int(success),
            'service_ok': int(jump_ok),
            'service_message': jump_message,
            'clearance_m': round(clearance, 4),
            'max_height_m': round(max_z, 4),
            'forward_progress_m': round(forward_progress, 4),
            'max_lateral_error_m': round(max_y_error, 4),
            'min_front_distance_m': round(min_front_distance, 4),
            'energy_proxy': round(energy_proxy / max(samples, 1), 4),
            'fell': int(fell),
        }


def write_outputs(rows, output_prefix):
    """Écrit les épisodes et un résumé statistique."""
    # Prépare les chemins de sortie demandés.
    prefix = Path(output_prefix)
    csv_path = prefix.with_suffix('.csv')
    json_path = prefix.with_suffix('.json')

    # Écrit les lignes détaillées en CSV.
    if rows:
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    # Calcule les statistiques principales.
    successes = sum(row['success'] for row in rows)
    summary = {
        'episodes': len(rows),
        'successes': successes,
        'success_rate': successes / max(len(rows), 1),
        'mean_clearance_m': sum(row['clearance_m'] for row in rows) / max(len(rows), 1),
        'mean_forward_progress_m': sum(row['forward_progress_m'] for row in rows) / max(len(rows), 1),
        'mean_energy_proxy': sum(row['energy_proxy'] for row in rows) / max(len(rows), 1),
        'episodes_detail': rows,
    }

    # Écrit le résumé JSON complet.
    json_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return csv_path, json_path, summary


def main():
    """Parse les arguments et exécute la campagne d'évaluation."""
    # Définit les options de campagne depuis la ligne de commande.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--success-height', type=float, default=0.20)
    parser.add_argument('--max-lateral-error', type=float, default=1.0)
    parser.add_argument('--reset-between-episodes', action='store_true')
    parser.add_argument('--output', default='jump_policy_evaluation')
    args = parser.parse_args()

    # Initialise ROS 2 et le nœud d'évaluation.
    rclpy.init()
    node = JumpPolicyEvaluator()
    rows = []

    try:
        # Attend une première odométrie avant de commencer.
        if not node.wait_for_topics(timeout=10.0):
            raise RuntimeError('Aucune odométrie reçue sur /odom')

        # Exécute les épisodes demandés.
        for episode_id in range(1, args.episodes + 1):
            if args.reset_between_episodes:
                node.reset_world(timeout=3.0)
                time.sleep(0.25)
                node.wait_for_topics(timeout=3.0)
            result = node.run_episode(
                episode_id,
                args.duration,
                args.success_height,
                args.max_lateral_error,
            )
            rows.append(result)
            node.get_logger().info(
                f"Episode {episode_id}: success={result['success']} clearance={result['clearance_m']:.3f}m"
            )

        # Sauvegarde les résultats de la campagne.
        csv_path, json_path, summary = write_outputs(rows, args.output)
        node.get_logger().info(
            f"Success rate={summary['success_rate']:.2%}; CSV={csv_path}; JSON={json_path}"
        )
    finally:
        # Détruit le nœud et ferme ROS 2 proprement.
        node.destroy_node()
        rclpy.shutdown()


# Lance l'évaluation quand le fichier est exécuté comme script.
if __name__ == '__main__':
    main()
