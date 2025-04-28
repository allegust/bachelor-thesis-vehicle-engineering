from pathlib import Path
from typing import Tuple, Union

import gpxpy
import numpy as np
from pyproj import Transformer


def _circle_radius(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    """
    Compute the circumcircle radius of the triangle p1-p2-p3.
    Returns inf for degenerate triangles.
    """
    a = np.linalg.norm(p2 - p3)
    b = np.linalg.norm(p1 - p3)
    c = np.linalg.norm(p1 - p2)
    s = 0.5 * (a + b + c)
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0:
        return float('inf')
    area = np.sqrt(area_sq)
    return (a * b * c) / (4.0 * area)


def map_data(
    map_file: Union[str, Path],
    rr_coef_input: float,
    ay_max: float,
    temp_c: float,
    earth_radius: float,
    max_slope: float,
) -> Tuple[
    np.ndarray,  # step_dist
    np.ndarray,  # step_ele
    np.ndarray,  # slope_angle
    np.ndarray,  # step_rr
    np.ndarray,  # reduce_speed_dist
    np.ndarray,  # v_max_latacc
    np.ndarray,  # v_max_xroads
]:
    """
    Parse GPX and return per-step arrays:
      step_dist, step_ele, slope_angle, step_rr,
      reduce_speed_dist, v_max_latacc, v_max_xroads.
    All constants (earth_radius, max_slope) are passed in.
    """
    # 1) Load GPX
    with open(map_file, 'r', encoding='utf-8') as fh:
        gpx = gpxpy.parse(fh.read())

    lat, lon, ele = [], [], []
    for trk in gpx.tracks:
        for seg in trk.segments:
            for pt in seg.points:
                lat.append(pt.latitude)
                lon.append(pt.longitude)
                ele.append(pt.elevation)
    lat = np.asarray(lat, float)
    lon = np.asarray(lon, float)
    ele = np.asarray(ele, float)

    if lat.size < 2:
        empty = np.empty(0)
        return (empty,) * 7

    # 2) Horizontal distances via Haversine
    φ1 = np.deg2rad(lat[:-1])
    φ2 = np.deg2rad(lat[1:])
    dφ = φ2 - φ1
    dλ = np.deg2rad(lon[1:] - lon[:-1])
    a = np.sin(dφ * 0.5)**2 + np.cos(φ1) * np.cos(φ2) * np.sin(dλ * 0.5)**2
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    step_dist = earth_radius * c

    # 3) Elevation & slope
    step_ele = ele[1:]
    delta_ele = np.diff(ele)
    slope_angle = np.zeros_like(step_dist)
    mask = step_dist > 0
    slope_angle[mask] = np.arctan(delta_ele[mask] / step_dist[mask])
    np.clip(slope_angle, -max_slope, max_slope, out=slope_angle)

    # 4) Rolling resistance per step
    if rr_coef_input == 0:
        c_r = 0.274 / (temp_c + 46.8) + 0.0037
        step_rr = np.full_like(step_dist, c_r)
    else:
        lookup = np.array([
            3.29357798165138, 3.14912280701754, 3.01680672268908,
            2.89516129032258, 2.78294573643411, 2.67910447761194,
            2.58273381294964, 2.49305555555556, 2.40939597315436,
            2.33116883116883, 2.25786163522013, 2.18902439024390,
            2.12426035502959, 2.06321839080460, 2.00558659217877,
            1.95108695652174, 1.89947089947090, 1.85051546391753,
            1.80402010050251, 1.75980392156863, 1.71770334928230,
            1.67757009345794, 1.63926940639269, 1.60267857142857,
            1.56768558951965, 1.53418803418803, 1.50209205020921,
            1.47131147540984, 1.44176706827309, 1.41338582677165,
            1.38610038610039, 1.35984848484848, 1.33457249070632,
            1.31021897810219, 1.28673835125448, 1.26408450704225,
            1.24221453287197, 1.22108843537415, 1.20066889632107,
            1.18092105263158, 1.16181229773463, 1.14331210191083,
            1.12539184952978, 1.10802469135802, 1.09118541033435,
            1.07485029940120, 1.05899705014749, 1.04360465116279,
            1.02865329512894, 1.01412429378531, 1.0
        ])
        t_int = int(round(temp_c))
        if t_int > 25:
            factor = 1.0
        elif t_int < -25:
            factor = 3.5
        else:
            factor = lookup[t_int + 25]
        step_rr = np.full_like(step_dist, rr_coef_input * factor)

    # 5) Curvature → lateral‐acc limits
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True)
    x, y = transformer.transform(lon, lat)
    radii = []
    for j in range(2, len(x)):
        R = _circle_radius(
            np.array([x[j-2], y[j-2]]),
            np.array([x[j-1], y[j-1]]),
            np.array([x[j],   y[j]]),
        )
        radii.append(max(R, 1.0))
    radii.append(radii[-1])
    v_max_latacc = np.sqrt(ay_max * np.array(radii))

    # 6) Crossroads stub
    reduce_speed_dist = np.zeros_like(step_dist)
    cum_dist = np.cumsum(step_dist)
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
