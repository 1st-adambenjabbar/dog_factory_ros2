"""Tests pytest des comportements attendus de la perception LiDAR.

Ces tests utilisent des nuages de points synthétiques, car la compilation et
l'exécution d'un nœud ROS 2 complet nécessitent un environnement Humble actif.
La fonction de référence reproduit l'algorithme de segmentation du C++.
"""

# Importe les fonctions mathématiques utilisées pour les distances.
import math

# Importe pytest pour structurer les cas de test.
import pytest


def segment_points(points, cluster_distance=0.35, min_cluster_points=3):
    """Référence Python du regroupement par distance du nœud C++."""

    # Retourne une liste vide lorsqu'aucun point n'est disponible.
    if not points:
        return []

    # Commence le premier cluster avec le premier point.
    clusters = [[points[0]]]

    # Parcourt chaque point dans l'ordre angulaire du scan.
    for point in points[1:]:
        # Récupère le dernier point du cluster courant.
        previous = clusters[-1][-1]

        # Calcule la distance cartésienne entre les deux impacts.
        distance = math.hypot(point[0] - previous[0], point[1] - previous[1])

        # Ajoute le point si la séparation reste compatible avec le même obstacle.
        if distance <= cluster_distance:
            clusters[-1].append(point)
        else:
            # Sinon, démarre une nouvelle hypothèse d'obstacle.
            clusters.append([point])

    # Ne conserve que les obstacles ayant assez de mesures.
    return [cluster for cluster in clusters if len(cluster) >= min_cluster_points]


def front_distance(points, half_angle=0.45):
    """Retourne la distance minimale dans le secteur frontal."""

    # Conserve uniquement les points situés dans le cône avant.
    distances = [
        math.hypot(x, y)
        for x, y, angle in points
        if abs(angle) <= half_angle
    ]

    # Renvoie une valeur conventionnelle lorsqu'aucun point n'est présent.
    return min(distances) if distances else 12.0


def test_empty_scan_has_no_obstacle():
    """Un scan vide ne doit créer aucun cluster."""

    # Vérifie que l'absence de données est traitée sans exception.
    assert segment_points([]) == []


def test_three_near_points_form_one_obstacle():
    """Trois points proches doivent former un obstacle unique."""

    # Construit un petit nuage correspondant à une caisse proche.
    points = [(1.0, -0.1), (1.02, 0.0), (1.0, 0.1)]

    # Vérifie qu'un seul cluster est obtenu.
    clusters = segment_points(points)
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_two_separated_obstacles_form_two_clusters():
    """Deux groupes séparés doivent rester deux obstacles différents."""

    # Construit deux groupes avec un espace supérieur au seuil.
    points = [(1.0, 0.0), (1.05, 0.03), (1.1, 0.05), (2.0, 0.0), (2.05, 0.02), (2.1, 0.04)]

    # Vérifie la séparation des deux objets.
    clusters = segment_points(points)
    assert len(clusters) == 2


def test_small_noise_cluster_is_rejected():
    """Un seul ou deux points isolés doivent être filtrés."""

    # Ajoute un bruit isolé puis un vrai groupe de trois points.
    points = [(0.2, 0.2), (1.0, 0.0), (1.03, 0.02), (1.05, 0.04)]

    # Le bruit est séparé et ne satisfait pas le nombre minimal de points.
    clusters = segment_points(points)
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_front_obstacle_distance_is_detected():
    """La distance frontale doit être estimée sur le nuage synthétique."""

    # Construit des points avant et latéraux, comme un scan d'usine.
    points = [(0.8, 0.0, 0.0), (1.2, 0.2, 0.2), (0.5, 2.0, 1.2)]

    # Vérifie que le point central devient la distance dangereuse.
    assert front_distance(points) == pytest.approx(0.8)


def test_no_front_obstacle_returns_max_range():
    """Une scène sans point frontal doit renvoyer la portée conventionnelle."""

    # Place tous les points hors du secteur frontal.
    points = [(1.0, 2.0, 1.0), (1.0, -2.0, -1.0)]

    # Vérifie la valeur de repli de 12 mètres.
    assert front_distance(points) == pytest.approx(12.0)
