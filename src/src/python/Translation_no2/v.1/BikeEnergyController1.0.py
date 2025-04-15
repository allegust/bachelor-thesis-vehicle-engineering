import gpxpy
import numpy as np
import os

def find_repo_root():
    current_dir = os.path.abspath(os.path.dirname(__file__))
    while current_dir != "/" and not os.path.exists(os.path.join(current_dir, ".git")):
        current_dir = os.path.dirname(current_dir)
    return current_dir if os.path.exists(os.path.join(current_dir, ".git")) else None

REPO_ROOT = find_repo_root()
if REPO_ROOT is None:
    print("ERROR: Could not determine repository root. Make sure you're inside the repo.")
    exit(1)

DATA_DIR = os.path.join(REPO_ROOT, "data", "data", "raw")
GPX_FILE_NAME = "GraphHopper_Track_DD_Koenigsbruecker_up.gpx"
GPX_FILE_PATH = os.path.join(DATA_DIR, GPX_FILE_NAME)

def parse_gpx(GPX_FILE_PATH):
    with open(GPX_FILE_PATH, "r") as gpx_file:
        gpx = gpxpy.parse(gpx_file)

    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                points.append([point.latitude, point.longitude, point.elevation])

    points = np.array(points)
    distances = np.zeros(len(points))
    elevations = points[:, 2]

    # Calculate distances between points
    from geopy.distance import geodesic
    for i in range(1, len(points)):
        distances[i] = distances[i-1] + geodesic(points[i-1, :2], points[i, :2]).meters

    return distances, elevations

distances, elevations = parse_gpx(GPX_FILE_PATH)


from scipy.stats import norm, weibull_min
from itertools import product

# example from MATLAB file
percentiles = np.arange(0.1, 1.0, 0.1)
VO2max_women = np.array([24.225, 27.6, 29.1, 30.975, 32.125, 33.625, 35.5, 37.8, 41.625])
PowerFactor = 71.8 / 1000  # W/(l/min VO2)

WeightSpecificPower_women = VO2max_women * PowerFactor

# Assuming weights and distributions from MATLAB
Weight_mean_women = np.mean([61.8, 64.7, 67.3, 68.9, 70.3, 71.3, 72.3, 73.1])
Weight_25_women = np.mean([54.8, 57.3, 59.8, 61.1, 62.5, 63.5, 64.5, 65.2])
Weight_75_women = np.mean([68.5, 71.8, 74.6, 76.8, 78.6, 79.8, 80.8, 81.6])

# Normal distribution weights
def normal_distribution_weight(mean, q25, q75):
    z25 = norm.ppf(0.25)
    z75 = norm.ppf(0.75)
    sigma = (q75 - q25) / (z75 - z25)
    return norm.ppf(percentiles, mean, sigma)

Weight_deciles_women = normal_distribution_weight(Weight_mean_women, Weight_25_women, Weight_75_women)

# Combine values (similar to MATLAB's combinations)
combinations_input = list(product(WeightSpecificPower_women * Weight_deciles_women, Weight_deciles_women,
                                  weibull_min.ppf(percentiles, 2.28, scale=0.00874),
                                  np.array([0.356, 0.402, 0.452, 0.493, 0.542, 0.588, 0.633, 0.695, 0.823])))

# Run your model over each combination here


