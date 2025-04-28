from pathlib import Path
import gpxpy
import numpy as np


def map_data(
    map_file_path: Union[str, Path, list],
    rr_coef_input: float,
    ay_max: float,
    temp_c: float,
) -> Tuple[np.ndarray, ...]:
    """Parse GPX or list of (lat, lon, ele) and return the seven step-arrays used elsewhere."""
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

    # If fewer than 3 points, return empties
    if lat.size < 3:
        empty = np.empty(0)
        return (empty,) * 7

    # Project lat/lon to planar coords (same as before)
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True)
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

    # Elevation and slope as in MATLAB version
    step_ele = ele[1:]
    dz0 = step_ele[0] - ele[0]
    step_dist = dist_2d
    slope_angle = np.arctan2(dz0, step_dist)
    np.clip(slope_angle, -0.2, 0.2, out=slope_angle)

    # Rolling resistance (unchanged)
    if rr_coef_input == 0:
        c_r = 0.274 / (temp_c + 46.8) + 0.0037
        step_rr = np.full_like(step_dist, c_r)
    else:
        temp_int = int(round(temp_c))
        # ... temperature lookup logic ...
        # (copy existing factor_lookup branch here)

    # Curvature-based lateral-acc limits (unchanged)
    n_pts = len(x)
    radii = []
    for j in range(2, n_pts):
        R = _circle_radius(np.array([x[j-2], y[j-2]]),
                           np.array([x[j-1], y[j-1]]),
                           np.array([x[j],   y[j]]))
        radii.append(max(R, 1.0))
    radii.append(radii[-1])
    v_max_latacc = np.sqrt(ay_max * np.array(radii))

    # Crossroads / stop logic table
    cum_dist = np.cumsum(step_dist)
    reduce_speed_dist = np.zeros_like(step_dist)
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
