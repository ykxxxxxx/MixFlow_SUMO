import numpy as np
from sklearn.cluster import DBSCAN

def perform_dbscan_clustering(vehicles, epsilon=4, min_samples=3):
    """
    Perform DBSCAN clustering on the given list of vehicles.

    Parameters
    ----------
    vehicles : list of Vehicle
        List of vehicles to cluster.
    epsilon : float, optional
        The maximum distance between two samples for them to be considered as in
        the same neighborhood. Defaults to 4.
    min_samples : int, optional
        The number of samples in a neighborhood for a point to be considered as
        a core point. This includes the point itself. Defaults to 3.

    Returns
    -------
    dict
        Dictionary containing the cluster labels for each vehicle. Vehicles
        that are not in any cluster are given a label of -1.
    """
    # Construct feature matrix
    features = np.zeros((len(vehicles), 2))
    for i, vehicle in enumerate(vehicles):
        features[i, 0] = vehicle.position[0]  # x-coordinate
        features[i, 1] = vehicle.velocity  # velocity

    # Perform DBSCAN clustering
    db = DBSCAN(eps=epsilon, min_samples=min_samples).fit(features)

    # Extract cluster labels
    labels = db.labels_

    # Construct dictionary of cluster labels
    clusters = {}
    for i, vehicle in enumerate(vehicles):
        if labels[i] == -1:
            clusters[vehicle] = []
        else:
            if labels[i] not in clusters:
                clusters[labels[i]] = []
            clusters[labels[i]].append(vehicle)

    return clusters
