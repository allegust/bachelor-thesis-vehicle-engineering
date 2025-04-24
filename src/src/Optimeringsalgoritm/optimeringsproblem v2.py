### optimeringsproblem_v2_updated.py

import math
import numpy as np
import matplotlib.pyplot as plt

from BikeEnergyOptimizedV2 import BikeEnergyModel   # lever‑1 engine

# --------------------------- parameters -----------------------------------
highway_width = 55        # [m]
max_slope     = 0.05      # ±5 %
start_height  = 1.0       # [m]
end_height    = 2.0       # [m]

LAT0, LON0 = 59.329, 18.068          # anchor for synthetic GPS (Stockholm)
M_PER_DEG  = 111_000.0               # 1° latitude ≈111 km

# ------------------------ helper functions --------------------------------

def as_gps(coords_xyz):
    """Convert local (x, y, z) [m] to (lat, lon, ele) [deg, deg, m]."""
    gps = []
    cos_lat = math.cos(math.radians(LAT0))
    for x, y, z in coords_xyz:
        lat = LAT0 + x / M_PER_DEG
        lon = LON0 + y / (M_PER_DEG * cos_lat)
        gps.append((lat, lon, z))
    return gps


def generate_profile(min_height, n_curve=120, n_flat=60):
    """Return (x, z) arrays for the over/under profile."""
    length_before = abs(min_height - start_height) / max_slope
    length_after  = abs(end_height - min_height) / max_slope

    highway_start = -highway_width / 2
    highway_end   =  highway_width / 2

    x_left  = np.linspace(-length_before - highway_width/2, highway_start, n_curve)
    x_flat  = np.linspace(highway_start,                    highway_end,   n_flat)
    x_right = np.linspace(highway_end,   length_after + highway_width/2,  n_curve)

    t_left  = (x_left  - x_left[0])  / (x_left[-1]  - x_left[0])
    t_right = (x_right - x_right[0]) / (x_right[-1] - x_right[0])

    z_left  = start_height + (min_height - start_height) * np.sin(np.pi*t_left /2)**2
    z_flat  = np.full_like(x_flat, min_height)
    z_right = min_height  + (end_height - min_height) * np.sin(np.pi*t_right/2)**2

    x_total = np.concatenate([x_left, x_flat, x_right])
    z_total = np.concatenate([z_left, z_flat, z_right])
    return x_total, z_total

# --------------------------- build profiles -------------------------------

x_over,  z_over  = generate_profile(min_height=5.0)
x_under, z_under = generate_profile(min_height=-3.0)   # enable if needed

coords_over_xyz  = list(zip(x_over,  np.zeros_like(x_over),  z_over))
coords_over_gps  = as_gps(coords_over_xyz)

coords_under_xyz = list(zip(x_under, np.zeros_like(x_under), z_under))
coords_under_gps = as_gps(coords_under_xyz)

# --------------------------- run energy model -----------------------------

POWER = 150.0     # W
MASS  = 70.0      # kg (rider)
C_R   = 0.007
CWXA  = 0.45

E_o, T_o, D_o, V_o = BikeEnergyModel(POWER, MASS, C_R, CWXA, coords_over_gps)
E_u, T_u, D_u, V_u = BikeEnergyModel(POWER, MASS, C_R, CWXA, coords_under_gps)

print("--- OVER THE HIGHWAY ---")
print(f"Energy: {E_o:.1f} J | Time: {T_o:.1f} s | Distance: {D_o:.1f} m | "
      f"Avg Speed: {V_o*3.6:.1f} km/h")
print("--- UNDER THE HIGHWAY ---")
print(f"Energy: {E_u:.1f} J | Time: {T_u:.1f} s | Distance: {D_u:.1f} m | "
     f"Avg Speed: {V_u*3.6:.1f} km/h")

# --------------------------- plotting -------------------------------------
plt.figure(figsize=(12, 6))
plt.plot(x_over, z_over, label="Over the highway")
plt.plot(x_under, z_under, label="Under the highway")

highway_start = -highway_width/2
highway_end   =  highway_width/2
plt.hlines(0, highway_start, highway_end, colors="grey", linewidth=5, label="Highway")
plt.scatter([x_over[0], x_over[-1]], [start_height, end_height], c="red", zorder=5)
plt.scatter([x_under[0], x_under[-1]], [start_height, end_height], c="red", zorder=5)

plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
plt.title("Cycle‑path profile over / under the highway")
plt.legend()
plt.grid(True)
plt.axhline(0, color="black", linewidth=0.5)
plt.axvline(0, color="black", linewidth=0.5)
plt.tight_layout()
plt.show()
