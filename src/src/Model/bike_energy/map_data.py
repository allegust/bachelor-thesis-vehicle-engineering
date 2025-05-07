from __future__ import annotations

import math
from pathlib import Path
from typing import Tuple, Union, List

import gpxpy
import numpy as np
from pyproj import Transformer


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
    # --- Handle direct coordinate input (list of tuples) ----------------
    if not isinstance(map_file_path, (str, Path)):
        coords = map_file_path
        lat = np.asarray([c[0] for c in coords], dtype=np.float64)
        lon = np.asarray([c[1] for c in coords], dtype=np.float64)
        ele = np.asarray([c[2] for c in coords], dtype=np.float64)
    else:
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

    

    """    delta_ele = ele[1:] - ele[:-1]
    step_dist = dist_2d                  # horizontal distance only
    step_ele  = ele[1:]                  # no elevation smoothing

    #Slope angle, clipped to ±0.2 rad -------------------------------------
    # Slope
    slope_angle = np.zeros_like(step_dist)
    mask = dist_2d > 0
    slope_angle[mask] = np.arctan(delta_ele[mask] / dist_2d[mask])
    np.clip(slope_angle, -0.2, 0.2, out=slope_angle)"""

    # keep the absolute elevation at the “end” of each segment
    # keep the end‐point elevations (for downstream use)
    step_ele = ele[1:]

    # compute every actual Δe for debugging (you know this is full array)
    delta_ele = step_ele - ele[:-1]

    # now force Python to use the MATLAB‐observed “first Δe = 1.0” for all segments
    dz0 = delta_ele[0]   # = ele[1] – ele[0]

    # horizontal run of each segment
    step_dist = dist_2d

    # exactly mimic the MATLAB StepAngle you saw:
    slope_angle = np.arctan2(dz0, step_dist)

    # clip as MapData.m does
    np.clip(slope_angle, -0.2, 0.2, out=slope_angle)
    #print("first 10 slopes:", slope_angle[:10])

    """    print("first 10 dists:", step_dist[:10])
    print("first 10 slopes:", slope_angle[:10])
    print("sum dist:", step_dist.sum())"""

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