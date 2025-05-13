import os, sys
from pathlib import Path

# Ensure we run from this folder and can import bike_energy
root = Path(__file__).parent
os.chdir(root)
sys.path.insert(0, str(root))

import math
import numpy as np
import matplotlib.pyplot as plt

# pull in all our tunables from params_opt.py
from params_optpimization import (
    ambient_temp as T,
    highway_width as w,
    max_slope    as s,
    start_height as h0,
    end_height   as h1,
    gps_anchor,
    power        as P,
    mass         as M,
    c_r          as C_R,
    cwxA         as CWXA,
    min_height_over,
    min_height_under,
    n_curve,
    n_flat,
)

# bring in all the shared constants your sim needs
from bike_energy.config import (
    DRIVETRAIN_LOSS,
    V_MAX,
    AX_ADAPT,
    AX_LATACC,
    AIR_DENSITY_A,
    AIR_DENSITY_B, 
    AIR_DENSITY_C,
    AY_MAX,
)

from bike_energy.map_data import map_data
from bike_energy.free_rolling import free_rolling_slope
from bike_energy.simulator import simulate_energy

def as_gps(coords_xyz, lat0, lon0):
    """Convert local (x,y,z) to (lat,lon,elevation)."""
    M_PER_DEG = 111_000.0
    return [
        (
            lat0 + x / M_PER_DEG,
            lon0 + y / (M_PER_DEG * math.cos(math.radians(lat0))),
            z,
        )
        for x, y, z in coords_xyz
    ]

def main():
    # ─── Unpack GPS anchor ───────────────────────────────────────────────────
    lat0 = gps_anchor["lat0"]
    lon0 = gps_anchor["lon0"]

    # ─── Helpers to generate & pad profiles ─────────────────────────────────
    def generate_profile(min_h: float, n_curve: int, n_flat: int):
        L_before = abs(min_h - h0) / s
        L_after  = abs(h1 - min_h) / s

        x1 = np.linspace(-L_before - w/2, -w/2, n_curve)
        t1 = (x1 - x1[0]) / (x1[-1] - x1[0])
        z1 = h0 + (min_h - h0) * np.sin(np.pi * t1 / 2) ** 2

        x2 = np.linspace(-w/2, +w/2, n_flat)
        z2 = np.full_like(x2, min_h)

        x3 = np.linspace(+w/2, L_after + w/2, n_curve)
        t3 = (x3 - x3[0]) / (x3[-1] - x3[0])
        z3 = min_h + (h1 - min_h) * np.sin(np.pi * t3 / 2) ** 2

        return np.concatenate([x1, x2, x3]), np.concatenate([z1, z2, z3])

    def pad_profile(x: np.ndarray, z: np.ndarray, x0: float, x1: float, n_pad: int = 10):
        x_start, x_end = x[0], x[-1]
        if x_start > x0:
            xp = np.linspace(x0, x_start, n_pad, endpoint=False)
            x = np.concatenate([xp, x])
            z = np.concatenate([np.full_like(xp, z[0]), z])
        if x_end < x1:
            xp = np.linspace(x_end, x1, n_pad+1)[1:]
            x = np.concatenate([x, xp])
            z = np.concatenate([z, np.full_like(xp, z[-1])])
        return x, z

    # ─── Build and pad profiles ──────────────────────────────────────────────
    x_over,  z_over  = generate_profile(min_height_over,  n_curve, n_flat)
    x_under, z_under = generate_profile(min_height_under, n_curve, n_flat)
    x0 = min(x_over[0], x_under[0])
    x1 = max(x_over[-1], x_under[-1])
    x_over,  z_over  = pad_profile(x_over,  z_over,  x0, x1)
    x_under, z_under = pad_profile(x_under, z_under, x0, x1)

    # ─── Convert to GPS and map_data steps ──────────────────────────────────
    coords_over_gps  = as_gps(list(zip(x_over,  np.zeros_like(x_over),  z_over)), lat0, lon0)
    coords_under_gps = as_gps(list(zip(x_under, np.zeros_like(x_under), z_under)), lat0, lon0)

    sd_o, se_o, sa_o, rr_o, red_o, vla_o, vxr_o = map_data(
        coords_over_gps, C_R, AY_MAX, T
    )
    sd_u, se_u, sa_u, rr_u, red_u, vla_u, vxr_u = map_data(
        coords_under_gps, C_R, AY_MAX, T
    )

    # ─── Compute steady-state alpha at V_MAX ────────────────────────────────
    rho = AIR_DENSITY_A*T**2 + AIR_DENSITY_B*T + AIR_DENSITY_C
    m_tot = M + 18.3
    alpha_vec, vx_vec = free_rolling_slope(m_tot, rr_o[0], CWXA, rho)
    idx = int(np.argmin(np.abs(vx_vec - V_MAX)))
    alpha_ss = alpha_vec[idx]

    # ─── Simulate forward leg ───────────────────────────────────────────────
    p_flat = P - DRIVETRAIN_LOSS

    Eo_f, To_f, Do_f, Vo_f, *rest_o = simulate_energy(
        sd_o, sa_o, rr_o, vla_o, vxr_o,
        m_tot, p_flat, V_MAX, AX_ADAPT, AX_LATACC, CWXA, rho, alpha_ss
    )
    Eu_f, Tu_f, Du_f, Vu_f, *rest_u = simulate_energy(
        sd_u, sa_u, rr_u, vla_u, vxr_u,
        m_tot, p_flat, V_MAX, AX_ADAPT, AX_LATACC, CWXA, rho, alpha_ss
    )

    # ─── Print one-way results ─────────────────────────────────────────────
    print("--- ONE-WAY OVER THE HIGHWAY ---")
    print(
        f"Energy: {Eo_f:.1f} J | "
        f"Time:   {To_f:.1f} s | "
        f"Distance: {Do_f:.1f} m | "
        f"Avg Speed: {Vo_f*3.6:.1f} km/h"
    )
    print("--- ONE-WAY UNDER THE HIGHWAY ---")
    print(
        f"Energy: {Eu_f:.1f} J | "
        f"Time:   {Tu_f:.1f} s | "
        f"Distance: {Du_f:.1f} m | "
        f"Avg Speed: {Vu_f*3.6:.1f} km/h"
    )

    # ─── Build return-leg coords & steps ────────────────────────────────────
    coords_over_back = [(2*x_over[-1] - x, 0, z) for (x, _, z) in zip(x_over, x_over, z_over[::-1])]
    coords_under_back= [(2*x_under[-1] - x, 0, z) for (x, _, z) in zip(x_under, x_under, z_under[::-1])]

    coords_over_gps_b  = as_gps(coords_over_back,  lat0, lon0)
    coords_under_gps_b = as_gps(coords_under_back, lat0, lon0)

    sd_ob, se_ob, sa_ob, rr_ob, red_ob, vla_ob, vxr_ob = map_data(
        coords_over_gps_b, C_R, AY_MAX, T
    )
    sd_ub, se_ub, sa_ub, rr_ub, red_ub, vla_ub, vxr_ub = map_data(
        coords_under_gps_b, C_R, AY_MAX, T
    )

    Eo_b, To_b, Do_b, Vo_b, *_ = simulate_energy(
        sd_ob, sa_ob, rr_ob, vla_ob, vxr_ob,
        m_tot, p_flat, V_MAX, AX_ADAPT, AX_LATACC, CWXA, rho, alpha_ss
    )
    Eu_b, Tu_b, Du_b, Vu_b, *_ = simulate_energy(
        sd_ub, sa_ub, rr_ub, vla_ub, vxr_ub,
        m_tot, p_flat, V_MAX, AX_ADAPT, AX_LATACC, CWXA, rho, alpha_ss
    )

    # ─── Sum for roundtrip and print ────────────────────────────────────────
    E_over = Eo_f + Eo_b
    T_over = To_f + To_b
    D_over = Do_f + Do_b
    V_over = D_over / T_over

    E_under = Eu_f + Eu_b
    T_under = Tu_f + Tu_b
    D_under = Du_f + Du_b
    V_under = D_under / T_under

    print(f"--- ROUNDTRIP OVER THE HIGHWAY ---")
    print(f"Energy: {E_over:.1f} J | Time: {T_over:.1f} s | Distance: {D_over:.1f} m | Avg Speed: {V_over*3.6:.1f} km/h")
    print(f"--- ROUNDTRIP UNDER THE HIGHWAY ---")
    print(f"Energy: {E_under:.1f} J | Time: {T_under:.1f} s | Distance: {D_under:.1f} m | Avg Speed: {V_under*3.6:.1f} km/h")

    # ─── Plot profiles ───────────────────────────────────────────────────────
    plt.figure(figsize=(12, 6))
    plt.plot(x_over,  z_over,  label="Over the highway")
    plt.plot(x_under, z_under, label="Under the highway")
    plt.hlines(0, -w/2, +w/2, linewidth=5, label="Highway", color="grey")
    plt.scatter([x_over[0], x_over[-1]], [h0, h1], c="red", zorder=5)
    plt.scatter([x_under[0], x_under[-1]], [h0, h1], c="red", zorder=5)
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.title("Cycle-path profile over / under the highway")
    plt.legend()
    plt.grid(True)
    plt.axhline(0, color="black", linewidth=0.5)
    plt.axvline(0, color="black", linewidth=0.5)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
