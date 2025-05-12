#!/usr/bin/env python3
"""
run_optimization.py

Compare over/underpass cyclist energy using modular bike_energy package only.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from bike_energy.map_data import map_data
from bike_energy.free_rolling import free_rolling_slope
from bike_energy.simulator import simulate_energy
from bike_energy.config import (
    AY_MAX,
    DRIVETRAIN_LOSS,
    V_MAX,
    AX_ADAPT,
    AX_LATACC,
    AIR_DENSITY_A,
    AIR_DENSITY_B,
    AIR_DENSITY_C,
)

# Fixed test parameters
LAT0, LON0 = 59.329, 18.068
M_PER_DEG = 111_000.0
HIGHWAY_WIDTH = 55
MAX_SLOPE = 0.05
START_HEIGHT = 4
END_HEIGHT = -1
POWER = 150.0
MASS = 70.0
CR = 0.007
CWXA = 0.45


def as_gps(coords_xyz):
    gps = []
    cos_lat = math.cos(math.radians(LAT0))
    for x, y, z in coords_xyz:
        lat = LAT0 + x / M_PER_DEG
        lon = LON0 + y / (M_PER_DEG * cos_lat)
        gps.append((lat, lon, z))
    return gps


def generate_profile(min_height, n_curve=120, n_flat=60):
    length_before = abs(min_height - START_HEIGHT) / MAX_SLOPE
    length_after = abs(END_HEIGHT - min_height) / MAX_SLOPE

    highway_start = -HIGHWAY_WIDTH / 2
    highway_end = HIGHWAY_WIDTH / 2

    x_left, z_left = np.array([]), np.array([])
    if length_before > 0:
        x_left = np.linspace(-length_before - HIGHWAY_WIDTH/2, highway_start, n_curve)
        t_left = (x_left - x_left[0]) / (x_left[-1] - x_left[0])
        z_left = START_HEIGHT + (min_height - START_HEIGHT) * np.sin(np.pi * t_left / 2)**2

    x_flat = np.linspace(highway_start, highway_end, n_flat)
    z_flat = np.full_like(x_flat, min_height)

    x_right, z_right = np.array([]), np.array([])
    if length_after > 0:
        x_right = np.linspace(highway_end, length_after + HIGHWAY_WIDTH/2, n_curve)
        t_right = (x_right - x_right[0]) / (x_right[-1] - x_right[0])
        z_right = min_height + (END_HEIGHT - min_height) * np.sin(np.pi * t_right / 2)**2

    x_total = np.concatenate([x_left, x_flat, x_right])
    z_total = np.concatenate([z_left, z_flat, z_right])
    return x_total, z_total


def pad_profile(x, z, x_start_ref, x_end_ref, n_pad=10):
    x_new, z_new = x.copy(), z.copy()
    if x[0] > x_start_ref:
        x_pad = np.linspace(x_start_ref, x[0], n_pad, endpoint=False)
        z_pad = np.full_like(x_pad, z[0])
        x_new = np.concatenate([x_pad, x_new])
        z_new = np.concatenate([z_pad, z_new])
    if x[-1] < x_end_ref:
        x_pad = np.linspace(x[-1], x_end_ref, n_pad + 1)[1:]
        z_pad = np.full_like(x_pad, z[-1])
        x_new = np.concatenate([x_new, x_pad])
        z_new = np.concatenate([z_new, z_pad])
    return x_new, z_new


def simulate_route(coords_gps):
    temp = 20.0
    rho = AIR_DENSITY_A * temp ** 2 + AIR_DENSITY_B * temp + AIR_DENSITY_C
    total_mass = MASS + 18.3
    p_flat = POWER - DRIVETRAIN_LOSS

    step_dist, step_ele, slope_angle, step_rr, _, v_max_latacc, v_max_xroads = map_data(
        coords_gps, CR, AY_MAX, temp
    )

    if step_dist.size == 0:
        return 0, 0, 0, 0

    alpha_vec, vx_vec = free_rolling_slope(total_mass, step_rr[0], CWXA, rho)
    idx = int(np.argmin(np.abs(vx_vec - V_MAX)))
    alpha_vmax_ss = alpha_vec[idx]

    E, T, D, V = simulate_energy(
        step_dist,
        slope_angle,
        step_rr,
        v_max_latacc,
        v_max_xroads,
        total_mass,
        p_flat,
        V_MAX,
        AX_ADAPT,
        AX_LATACC,
        CWXA,
        rho,
        alpha_vmax_ss,
    )
    return E, T, D, V


def main():
    x_over, z_over = generate_profile(min_height=5.0)
    x_under, z_under = generate_profile(min_height=-3.0)

    x_start_common = min(x_over[0], x_under[0])
    x_end_common = max(x_over[-1], x_under[-1])

    x_over, z_over = pad_profile(x_over, z_over, x_start_common, x_end_common)
    x_under, z_under = pad_profile(x_under, z_under, x_start_common, x_end_common)

    coords_over = as_gps(list(zip(x_over, np.zeros_like(x_over), z_over)))
    coords_under = as_gps(list(zip(x_under, np.zeros_like(x_under), z_under)))

    E_o, T_o, D_o, V_o = simulate_route(coords_over)
    E_u, T_u, D_u, V_u = simulate_route(coords_under)

    print("--- OVER THE HIGHWAY ---")
    print(f"Energy: {E_o:.1f} J | Time: {T_o:.1f} s | Distance: {D_o:.1f} m | Avg Speed: {V_o*3.6:.2f} km/h")
    print("--- UNDER THE HIGHWAY ---")
    print(f"Energy: {E_u:.1f} J | Time: {T_u:.1f} s | Distance: {D_u:.1f} m | Avg Speed: {V_u*3.6:.2f} km/h")

    plt.figure(figsize=(10, 5))
    plt.plot(x_over, z_over, label="Over the highway")
    plt.plot(x_under, z_under, label="Under the highway")
    plt.axhline(0, color="gray", linewidth=5, label="Highway")
    plt.xlabel("Distance (m)")
    plt.ylabel("Elevation (m)")
    plt.title("Cycle Path Profile: Over vs Under Highway")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
