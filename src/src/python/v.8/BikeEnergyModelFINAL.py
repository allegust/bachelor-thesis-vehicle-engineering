###############################################################################
# BikeEnergyModelOPT_fast.py – Same physics, significantly less Python overhead
###############################################################################
from __future__ import annotations

import math
from pathlib import Path
from typing import Tuple, Union, List

import gpxpy
import numpy as np
from pyproj import Transformer

# --- external helper functions (unchanged physics) -------------------------
from SpeedReductionCausedByHighLatAcc import (
    Speed_Reduction_Caused_By_High_Lat_Acc as sr_latacc,
)
from SpeedReductionCausedByCrossRoads import (
    Speed_Reduction_Caused_By_CrossRoads as sr_xroad,
)
from PowerDeceleration import Power_Deceleration
from PowerInputOff import Power_Input_Off
from PowerInputOn import Power_Input_On
from FreeRollingSlope import Free_Rolling_Slope

# ---------------------------------------------------------------------------
STEP_SIZE = 0.01  # metres (1 cm) – global constant
INV_STEP = 1.0 / STEP_SIZE  # multiply instead of repeated division
G = 9.81  # m/s² – gravity, used in helpers but useful to bind locally

# ===========================================================================
# 1. Vectorised GPX‑file pre‑processing
# ===========================================================================

def _circle_radius(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    # Compute circumcircle radius of triangle p1-p2-p3
    a = np.linalg.norm(p2 - p3)
    b = np.linalg.norm(p1 - p3)
    c = np.linalg.norm(p1 - p2)
    # Heron's formula for area
    s = 0.5 * (a + b + c)
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0:
        return float('inf')
    area = math.sqrt(area_sq)
    return (a * b * c) / (4.0 * area)

def map_data(
    map_file_path: Union[str, Path],
    rr_coef_input: float,
    ay_max: float,
    temp_c: float,
) -> Tuple[np.ndarray, ...]:
    """Parse GPX and return the seven step‑arrays used elsewhere, now including
    curvature-based lateral-acc limits and proper crossroads table."""
    # Read GPX
    with open(map_file_path, "r", encoding="utf-8") as fh:
        gpx = gpxpy.parse(fh.read())

    # Flatten points
    lat, lon, ele = [], [], []
    for trk in gpx.tracks:
        for seg in trk.segments:
            for p in seg.points:
                lat.append(p.latitude)
                lon.append(p.longitude)
                ele.append(p.elevation)
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    ele = np.asarray(ele, dtype=np.float64)

    if lat.size < 3:
        # Return empty arrays as before
        empty = np.empty(0)
        return (empty,) * 7

    # Project lat/lon to planar coords (Web Mercator)
    #transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True)   # Same as in MATLAB
    x, y = transformer.transform(lon, lat)

    # Haversine for 2D distances
    lat1 = np.deg2rad(lat[:-1])
    lat2 = np.deg2rad(lat[1:])
    dlat = lat2 - lat1
    dlon = np.deg2rad(lon[1:] - lon[:-1])
    a = (np.sin(dlat / 2.0)**2
         + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    dist_2d = 6_371_000.0 * c

    delta_ele = ele[1:] - ele[:-1]
    step_dist = dist_2d                  # horizontal distance only
    step_ele  = ele[1:]                  # no elevation smoothing

    # Slope angle, clipped to ±0.2 rad -------------------------------------
    # Slope
    slope_angle = np.zeros_like(step_dist)
    mask = dist_2d > 0
    slope_angle[mask] = np.arctan(delta_ele[mask] / dist_2d[mask])
    np.clip(slope_angle, -0.2, 0.2, out=slope_angle)

    # Rolling resistance ----------------------------------------------------
    if rr_coef_input == 0:                                      # MATLAB default branch
        c_r = 0.274 / (temp_c + 46.8) + 0.0037
        step_rr = np.full_like(step_dist, c_r)
    else:                                                        # user-supplied Cr
        temp_int = int(round(temp_c))                            # °C → integer
        if   temp_int >  25: factor = 1.0
        elif temp_int < -25: factor = 3.5
        else:                                                    # -25 … 25 °C lookup
            factor_lookup = np.array([
                3.29357798165138, 3.14912280701754, 3.01680672268908, 2.89516129032258,
                2.78294573643411, 2.67910447761194, 2.58273381294964, 2.49305555555556,
                2.40939597315436, 2.33116883116883, 2.25786163522013, 2.18902439024390,
                2.12426035502959, 2.06321839080460, 2.00558659217877, 1.95108695652174,
                1.89947089947090, 1.85051546391753, 1.80402010050251, 1.75980392156863,
                1.71770334928230, 1.67757009345794, 1.63926940639269, 1.60267857142857,
                1.56768558951965, 1.53418803418803, 1.50209205020921, 1.47131147540984,
                1.44176706827309, 1.41338582677165, 1.38610038610039, 1.35984848484848,
                1.33457249070632, 1.31021897810219, 1.28673835125448, 1.26408450704225,
                1.24221453287197, 1.22108843537415, 1.20066889632107, 1.18092105263158,
                1.16181229773463, 1.14331210191083, 1.12539184952978, 1.10802469135802,
                1.09118541033435, 1.07485029940120, 1.05899705014749, 1.04360465116279,
                1.02865329512894, 1.01412429378531, 1.0
            ])
            factor = factor_lookup[temp_int + 25]                # index shift
        rr_val  = rr_coef_input * factor
        step_rr = np.full_like(step_dist, rr_val)

# ---------- curvature-based lateral-acceleration limits ----------
    n_pts = len(x)
    radii = []
    for j in range(2, n_pts):
        R = _circle_radius(np.array([x[j-2], y[j-2]]),
                           np.array([x[j-1], y[j-1]]),
                           np.array([x[j],   y[j]]))
        radii.append(max(R, 1.0))
    # radii length = n_pts - 2; append last
    radii.append(radii[-1])  # now length n_pts-1 == step_dist length
    v_max_latacc = np.sqrt(ay_max * np.array(radii))

    # --------- crossroads / stop logic table (defaults to no stops) ---------
    cum_dist = np.cumsum(step_dist)
    reduce_speed_dist = np.zeros_like(step_dist)  # user can customize this array
    v_max_xroads = np.column_stack((cum_dist, reduce_speed_dist))

    return (
        step_dist,
        step_ele,
        slope_angle,
        step_rr,
        reduce_speed_dist,
        v_max_latacc,
        v_max_xroads,
    )


# ===========================================================================
# 2 + 4 + 5. Optimised simulator (same mathematics)
# ===========================================================================

def simulate_energy(
    step_dist: np.ndarray,
    slope_angle: np.ndarray,
    step_rr: np.ndarray,
    v_max_latacc: np.ndarray,
    v_max_xroads: np.ndarray,
    m: float,
    p_flat: float,
    #p_up: float,       # Not used in this version
    v_max: float,
    ax_dec_adapt: float,
    ax_dec_latacc: float,
    cwxA: float,
    rho: float,
    alpha_vmax_ss: float,
):
    """Inner centimetre loop – rewritten to avoid Python overhead."""
    steps = step_dist.size
    time_s = 0.0
    energy = energy_roll = energy_air = energy_climb = energy_acc = 0.0

    vx = 5.0  # initial speed [m/s]
    v_x_total: List[np.ndarray] = []  # collect segments, concatenate at end

    for i in range(steps):
        dist_cm = int(round(step_dist[i] * INV_STEP))  # number of 1 cm slices
        if dist_cm == 0:
            continue

        v_seg = np.empty(dist_cm, dtype=np.float64)  # pre‑allocate (item 4)
        ang = slope_angle[i]
        rr_coef = step_rr[i]

        for ll in range(dist_cm):
            # ---- helper 1: lateral‑acc check ---------------------------------
            reduce_lat = sr_latacc(
                vx,
                ax_dec_latacc,
                v_max_latacc,
                dist_cm,
                steps,
                step_dist,
                ll,
                i,
            )

            # ---- helper 2: cross‑roads / stop logic --------------------------
            (
                red_status,
                v_x_target,
                ax_temp,
                stop_flag,
                _,
            ) = sr_xroad(
                vx,
                ax_dec_adapt,
                v_max_xroads,
                dist_cm,
                steps,
                step_dist,
                ll,
                i,
                v_x_total,          # real logs – helper only uses len()
                v_seg[:ll],         # current-segment part that is done
            )

            # ---- decide power / acceleration --------------------------------
            if red_status == 1 or stop_flag == 1:
                p_in, p_roll, p_air, p_climb, p_acc = Power_Deceleration(
                    m, rr_coef, ang, vx, cwxA, rho, p_flat, ax_temp
                )
            elif reduce_lat == 1:
                ax_temp = ax_dec_latacc
                p_in, p_roll, p_air, p_climb, p_acc = Power_Deceleration(
                    m, rr_coef, ang, vx, cwxA, rho, p_flat, ax_temp
                )
            elif red_status == 2:
                ax_temp = Power_Input_Off(m, rr_coef, ang, vx, cwxA, rho)
                p_in = p_roll = p_air = p_climb = p_acc = 0.0
            else:
                if ang <= alpha_vmax_ss:
                    # downhill or flat enough for free‑rolling limit
                    if vx > v_max:
                        ax_temp = ax_dec_adapt
                        p_in = p_roll = p_air = p_climb = p_acc = 0.0
                    elif vx == v_max:                       #elif abs(vx - v_max) < 1e-09:
                        # almost exactly at v_max – power off
                        p_in = 0.0
                        ax_temp = Power_Input_Off(
                            m, rr_coef, ang, vx, cwxA, rho
                        )
                        p_roll = p_air = p_climb = p_acc = 0.0
                    else:
                        (
                            ax_temp,
                            p_in,
                            p_roll,
                            p_air,
                            p_climb,
                            p_acc,
                        ) = Power_Input_On(
                            m, rr_coef, ang, vx, cwxA, rho, p_flat, v_max
                        )
                else:
                    # climbing or steep terrain section
                    if vx > v_max:
                        p_in = 0.0
                        ax_temp = Power_Input_Off(
                            m, rr_coef, ang, vx, cwxA, rho
                        )
                        p_roll = p_air = p_climb = p_acc = 0.0
                    else:
                        (
                            ax_temp,
                            p_in,
                            p_roll,
                            p_air,
                            p_climb,
                            p_acc,
                        ) = Power_Input_On(
                            m, rr_coef, ang, vx, cwxA, rho, p_flat, v_max
                        )

            # ---- kinematics & energy integration ----------------------------
            if ax_temp == 0.0:
                # micro‑opt (item 5) – skip sqrt if velocity unchanged
                new_vx = vx
            else:
                new_vx_sq = vx * vx + 2.0 * ax_temp * STEP_SIZE
                new_vx = math.sqrt(new_vx_sq) if new_vx_sq > 0.0 else 0.0

            if new_vx > 0.0:
                dt = STEP_SIZE / new_vx  # time for this cm
                time_s += dt
                energy += p_in * dt
                energy_roll += p_roll * dt
                energy_air += p_air * dt
                energy_climb += p_climb * dt
                energy_acc += p_acc * dt

            vx = new_vx
            v_seg[ll] = vx
        # end cm‑loop --------------------------------------------------------

        v_x_total.append(v_seg)
    # end segment‑loop -------------------------------------------------------

    v_x_total_arr = np.concatenate(v_x_total) if v_x_total else np.empty(0)
    distance = v_x_total_arr.size * STEP_SIZE
    avg_speed = distance / time_s if time_s > 0.0 else 0.0

    return (
        energy,
        time_s,
        distance,
        avg_speed,
        energy_roll,
        energy_air,
        energy_climb,
        energy_acc,
    )

# ===========================================================================
# Convenience wrapper – signature identical to the old BikeEnergyModel()
# ===========================================================================

def BikeEnergyModel(
    CyclistPowerIn: float,
    CyclistMassIn: float,
    CrIn: float,
    cwxA: float,
    map_file_path: Union[str, Path] = "my_route.gpx",
):
    temp = 20.0  # °C
    rho = (10 ** -5) * temp * temp - 0.0048 * temp + 1.2926

    V_max = 10.5
    Ay_max = 2.0
    ax_dec_adapt = -0.3
    ax_dec_latacc = -1.5

    (
        step_dist,
        step_ele,
        slope_angle,
        step_rr,
        _,
        v_max_latacc,
        v_max_xroads,
    ) = map_data(map_file_path, CrIn, Ay_max, temp)

    if step_dist.size == 0:
        return 0.0, 0.0, 0.0, 0.0

    m = CyclistMassIn + 18.3  # rider + bike+luggage
    p_flat = CyclistPowerIn - 5.0
    p_up = CyclistPowerIn - 5.0          # p_up = CyclistPowerIn * 1.5 - 5.0   Can be adjusted

    alpha_vec, vx_vec = Free_Rolling_Slope(m, step_rr[0], cwxA, rho)
    alpha_vmax_ss = alpha_vec[np.argmin(np.abs(np.asarray(vx_vec) - V_max))]

    E, T, Dist, Vavg, *_ = simulate_energy(
        step_dist,
        slope_angle,
        step_rr,
        v_max_latacc,
        v_max_xroads,
        m,
        p_flat,
        #p_up,       # Not used in this version
        V_max,
        ax_dec_adapt,
        ax_dec_latacc,
        cwxA,
        rho,
        alpha_vmax_ss,
    )

    return E, T, Dist, Vavg

# ---------------------------------------------------------------------------
if __name__ == "__main__":
    test_gpx = Path("my_route.gpx")
    res = BikeEnergyModel(150.0, 70.0, 0.007, 0.45, map_file_path=test_gpx)
    print("Energy [J]:", res[0])
    print("Time   [s]:", res[1])
    print("Dist   [m]:", res[2])
    print("Avg v  [m/s]:", res[3], "/", res[3] * 3.6, "km/h")