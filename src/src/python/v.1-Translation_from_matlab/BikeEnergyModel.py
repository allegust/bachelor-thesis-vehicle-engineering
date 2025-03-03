import os
import gpxpy
import gpxpy.gpx
import numpy as np
from geopy.distance import geodesic

def bike_energy_model(cyclist_power, cyclist_mass, rolling_resistance, aerodynamics, FILE_PATH):
    print(f"📂 Loading GPX data from: {FILE_PATH}")
    
    # Constants
    g = 9.81  # Gravity [m/s²]
    temp = 20  # Temperature in Celsius
    rho = 10**(-5) * temp**2 - 0.0048 * temp + 1.2926  # Air density [kg/m³]

    # Rolling resistance coefficient (default asphalt)
    C_r = rolling_resistance

    V_max = 10.5  # Maximum speed [m/s]
    ax_dec_adapt = -0.3  # Speed adaptation deceleration [m/s²]

    # Load GPX data
    step_distances, step_elevations, step_angles, step_rr_coefs = load_map_data(FILE_PATH)
    if not step_distances:
        print("Error: No GPX data loaded. Check file path.")
        return None, None, None, None

    # Cyclist parameters
    m = cyclist_mass + 18.3  # Total mass (cyclist + bike + luggage) [kg]
    P = cyclist_power - 5  # Adjusted power due to drivetrain losses

    # Initialize variables
    energy, time, distance = 0, 0, 0
    vx = 5  # Initial speed [m/s]

    for step_dist, step_angle, step_rr in zip(step_distances, step_angles, step_rr_coefs):
        dist_steps = max(1, int(round(step_dist / 0.01)))  # Prevent zero steps
        
        for _ in range(dist_steps):
            # Compute forces acting on the cyclist
            F_gravity = m * g * np.sin(step_angle)
            F_roll = step_rr * m * g * np.cos(step_angle)
            F_aero = 0.5 * rho * aerodynamics * vx**2

            # Compute total resistance
            F_total = F_gravity + F_roll + F_aero

            # Compute acceleration (Newton’s second law)
            ax = (P / max(vx, 0.1)) - (F_total / m)

            # Update velocity
            new_vx_squared = vx**2 + 2 * ax * 0.01
            vx = max(0.01, np.sqrt(new_vx_squared)) if new_vx_squared > 0 else 0.01

            # Compute energy and time updates
            energy += P * 0.01 / max(vx, 0.1)  # Avoid zero division
            time += 0.01 / max(vx, 0.1)
            distance += 0.01

    # Compute average speed
    avg_speed = distance / time if time > 0 else 0
    return energy, time, distance, avg_speed


def load_map_data(FILE_PATH):
    """
    Parses a GPX file and extracts distance, elevation, angle, and rolling resistance data.

    :param FILE_PATH: Path to the GPX file
    :return: Lists of step distances, elevations, angles, and rolling resistance coefficients
    """
    try:
        with open(FILE_PATH, "r") as gpx_file:
            gpx = gpxpy.parse(gpx_file)

        step_distances = []
        step_elevations = []
        step_angles = []
        step_rr_coefs = []  # Placeholder for rolling resistance, assumed constant for now

        prev_point = None

        for track in gpx.tracks:
            for segment in track.segments:
                for point in segment.points:
                    lat, lon, elevation = point.latitude, point.longitude, point.elevation

                    if prev_point:
                        # Compute step distance
                        step_distance = geodesic(
                            (prev_point.latitude, prev_point.longitude), (lat, lon)
                        ).meters

                        # Ensure step distance is valid
                        step_distance = max(step_distance, 0.01)

                        # Compute elevation change and slope angle
                        elevation_change = elevation - prev_point.elevation
                        step_angle = np.arctan(elevation_change / step_distance) if step_distance > 0 else 0

                        # Append data
                        step_distances.append(step_distance)
                        step_elevations.append(elevation_change)
                        step_angles.append(step_angle)
                        step_rr_coefs.append(0.007)  # Default asphalt rolling resistance

                    prev_point = point

        return step_distances, step_elevations, step_angles, step_rr_coefs

    except Exception as e:
        print(f"Error reading GPX file: {e}")
        return [], [], [], []
