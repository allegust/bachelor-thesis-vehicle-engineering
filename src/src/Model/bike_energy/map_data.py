from __future__ import annotations

import math
from pathlib import Path
from typing import Tuple, Union

import gpxpy
import numpy as np
from pyproj import Transformer
from bike_energy.config import (
    EARTH_RADIUS,
    RR_TEMP_LOOKUP,
    MAX_SLOPE,
)


def _circle_radius(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    a = np.linalg.norm(p2 - p3)
    b = np.linalg.norm(p1 - p3)
    c = np.linalg.norm(p1 - p2)
    s = 0.5 * (a + b + c)
    area_sq = s * (s - a) * (s - b) * (s - c)
    return float('inf') if area_sq <= 0 else (a * b * c) / (4.0 * math.sqrt(area_sq))

def map_data(
    map_file_path: Union[str, Path],
    rr_coef_input: float,
    ay_max: float,
    temp_c: float,
) -> Tuple[np.ndarray, ...]:
    if not isinstance(map_file_path, (str, Path)):
        coords = map_file_path
        lat, lon, ele = zip(*coords)
    else:
        with open(map_file_path, "r", encoding="utf-8") as fh:
            gpx = gpxpy.parse(fh.read())
        lat, lon, ele = zip(*[(p.latitude, p.longitude, p.elevation)
                              for trk in gpx.tracks for seg in trk.segments for p in seg.points])

    lat, lon, ele = map(np.asarray, [lat, lon, ele])

    if lat.size < 3:
        return (np.empty(0),) * 7

    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True)
    x, y = transformer.transform(lon, lat)

    lat1, lat2 = np.deg2rad(lat[:-1]), np.deg2rad(lat[1:])
    dlat, dlon = lat2 - lat1, np.deg2rad(lon[1:] - lon[:-1])
    a = (np.sin(dlat / 2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2)
    dist_2d = EARTH_RADIUS * 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))

    step_ele = ele[1:]
    delta_ele = step_ele - ele[:-1]
    dz0 = delta_ele[0]

    step_dist = dist_2d
    dz0 = delta_ele[0]
    slope_angle = np.arctan2(dz0, step_dist)
    np.clip(slope_angle, -MAX_SLOPE, MAX_SLOPE, out=slope_angle)

    if rr_coef_input == 0:
        c_r = 0.274 / (temp_c + 46.8) + 0.0037
    else:
        temp_int = int(round(temp_c))
        factor = (
            1.0 if temp_int > 25 else
            3.5 if temp_int < -25 else
            RR_TEMP_LOOKUP[temp_int + 25]
        )
        c_r = rr_coef_input * factor

    step_rr = np.full_like(step_dist, c_r)

    radii = [_circle_radius(np.array([x[j-2], y[j-2]]),
                            np.array([x[j-1], y[j-1]]),
                            np.array([x[j], y[j]])) for j in range(2, len(x))]
    radii.append(radii[-1])
    v_max_latacc = np.sqrt(ay_max * np.maximum(radii, 1.0))

    cum_dist = np.cumsum(step_dist)
    reduce_speed_dist = np.zeros_like(step_dist)
    v_max_xroads = np.column_stack((cum_dist, reduce_speed_dist))

    return step_dist, step_ele, slope_angle, step_rr, reduce_speed_dist, v_max_latacc, v_max_xroads
