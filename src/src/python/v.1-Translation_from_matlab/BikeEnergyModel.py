import os
import gpxpy
import gpxpy.gpx
import numpy as np
from geopy.distance import geodesic

def bike_energy_model(cyclist_power, cyclist_mass, rolling_resistance, aerodynamics, FILE_PATH):
    print(f"📂 Loading GPX data from: {FILE_PATH}")
    g = 9.81  # Gravity
    R_earth = 6371 * 10**3  # Earth radius [m]
    temp = 20  # Temperature in Celsius
    rho = 10**(-5) * temp**2 - 0.0048 * temp + 1.2926  # Air density

    V_max = 10.5  # Maximum speed [m/s]
    Ay_max = 2  # Maximum lateral acceleration [m/s²]
    ax_dec_adapt = -0.3  # Speed adaptation deceleration [m/s²]
    ax_dec_lat_acc = -1.5  # Lateral acceleration deceleration [m/s²]
    
    # FIX: Now passing FILE_PATH to load_map_data()
    step_distances, step_elevations, step_angles, step_rr_coefs = load_map_data(FILE_PATH)
    if not step_distances:
        print("Error: No GPX data loaded. Check file path.")
        return None, None, None, None
    
    m = cyclist_mass + 18.3  # Total mass including bicycle and luggage
    P = cyclist_power - 5  # Adjusted power due to drivetrain losses
    P_up = cyclist_power * 1.5 - 5  # Increased power for uphill riding

    energy, time, distance = 0, 0, 0
    vx = 5  # Initial speed [m/s]
    
    for step_dist, step_angle, step_rr in zip(step_distances, step_angles, step_rr_coefs):
        dist_steps = int(round(step_dist / 0.01))  # Dela upp sträckan i 1 cm steg
        for _ in range(dist_steps):
            ax = ax_dec_adapt if step_angle > 0 else ax_dec_lat_acc

            # Beräkna uppdaterad hastighet och förhindra negativa värden
            vx_update = vx**2 + 2 * ax * 0.01

            if vx_update < 0:
                vx = 0  # Stoppa cykeln om beräkningen skulle leda till imaginära tal
            else:
                vx = max(0, np.sqrt(vx_update))  # Säkerställ att vx aldrig blir negativ eller NaN

            # Undvik division med 0 genom att använda max(vx, 0.01)
            energy += P * 0.01 / max(vx, 0.01)
            time += 0.01 / max(vx, 0.01)
            distance += 0.01

    # Beräkna medelhastighet, förhindra division med 0
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
