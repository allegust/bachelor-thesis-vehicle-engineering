###############################################################################
# BikeEnergyOptimizedV2.py
###############################################################################
import gpxpy
import math
import numpy as np
from numba import njit

# Import each function from its separate files, as in your MATLAB approach:
from SpeedReductionCausedByHighLatAcc import Speed_Reduction_Caused_By_High_Lat_Acc
from SpeedReductionCausedByCrossRoads import Speed_Reduction_Caused_By_CrossRoads
from PowerDeceleration import Power_Deceleration
from PowerInputOff import Power_Input_Off
from PowerInputOn import Power_Input_On
from FreeRollingSlope import Free_Rolling_Slope


def MapData(map_file_path_or_coords, FigStatus, RRcoef_input, Ay_max, temp):
    """Unchanged from your original file – trimmed for brevity."""
    if isinstance(map_file_path_or_coords, str):
        with open(map_file_path_or_coords, 'r', encoding='utf-8') as f:
            gpx_data = f.read()
        gpx = gpxpy.parse(gpx_data)
        points = []
        for trk in gpx.tracks:
            for seg in trk.segments:
                for p in seg.points:
                    points.append(p)
    else:
        class Point:
            def __init__(self, latitude, longitude, elevation):
                self.latitude  = latitude
                self.longitude = longitude
                self.elevation = elevation
        points = [Point(lat, lon, ele) for lat, lon, ele in map_file_path_or_coords]

    if len(points) < 2:
        return [], [], [], [], [], [], []

    R_earth = 6371000.0
    StepDist, StepElevation, StepAngle = [], [], []

    for i in range(len(points) - 1):
        lat1, lon1, ele1 = points[i].latitude,  points[i].longitude,  points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation

        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a    = math.sin(dLat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dLon/2)**2
        dist_2d   = 2 * R_earth * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        delta_ele = ele2 - ele1
        totaldist = math.hypot(dist_2d, delta_ele)  # 3‑D chord length

        StepDist.append(totaldist)
        StepElevation.append(ele2)

        slope = math.atan2(delta_ele, dist_2d) if dist_2d else 0.0
        StepAngle.append(max(-0.2, min(0.2, slope)))

    # Rolling resistance
    c_r = 0.274/(temp + 46.8) + 0.004 if RRcoef_input == 0 else RRcoef_input
    StepRRcoef = [c_r] * len(StepDist)

    ReduceSpeedDist = [0] * len(StepDist)
    chosen_ay = Ay_max[0] if isinstance(Ay_max, (list, tuple)) else Ay_max
    V_max_LatAcc = [math.sqrt(chosen_ay * 9.81)] * len(StepDist)

    # Cumulative distance + dummy status 1 (free rolling)
    dist_accum = 0.0
    V_max_XRoads = []
    for d in StepDist:
        dist_accum += d
        V_max_XRoads.append([dist_accum, 1])

    return (StepDist, StepElevation, StepAngle, StepRRcoef,
            ReduceSpeedDist, V_max_LatAcc, np.asarray(V_max_XRoads, dtype=float))

# ======================================================================
# 2 ────────────────────  FAST INNER KERNEL  ────────────────────────────
# ======================================================================

MAX_SUB_STEP = 0.1   # 5 cm – change here if you want another resolution

@njit(cache=True, fastmath=True)
def _step_params(segment_dist, max_sub=MAX_SUB_STEP):
    """Return (step_size, n_steps) for a given GPX chord distance."""
    n_steps = int(math.ceil(segment_dist / max_sub))
    return segment_dist / n_steps, n_steps

@njit(cache=True, fastmath=True)
def update_velocity_and_energy(vx, ax, ds, P_in, P_roll, P_air, P_climb, P_acc):
    new_v2 = vx*vx + 2.0*ax*ds
    vx_new = math.sqrt(new_v2) if new_v2 > 0.0 else 0.0
    if vx_new == 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    dt = ds / vx_new
    return (vx_new, dt,  P_in*dt, P_roll*dt, P_air*dt,
            P_climb*dt, P_acc*dt)

# ------------------------------------------------------------------
# simulate_energy: **only change is adaptive sub‑step logic**
# ------------------------------------------------------------------

def simulate_energy(StepDist, StepAngle, StepRRcoef,
                    V_max_LatAcc, V_max_XRoads,
                    m, P_flat, P_up, V_max,
                    ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho,
                    Alpha_Vmax_steadystate):

    Steps = len(StepDist)
    Time = Energy = Energy_roll = Energy_air = Energy_climb = Energy_acc = 0.0
    vx = 5.0  # initial speed [m/s]
    distance_done = 0.0

    for i in range(Steps):
        seg_dist   = StepDist[i]
        step_angle = StepAngle[i]
        step_rr    = StepRRcoef[i]

        step_size, n_steps = _step_params(seg_dist)   # ← adaptive break‑up

        # loop through the sub‑steps of this GPX segment
        for ll in range(n_steps):
            reduce_lat = Speed_Reduction_Caused_By_High_Lat_Acc(
                vx, ax_Dec_LatAcc, V_max_LatAcc,
                n_steps, Steps, StepDist, ll, i)

            redSpdStatus, v_x_target, ax_temp, StopFlag, _ = Speed_Reduction_Caused_By_CrossRoads(
                vx, ax_Dec_adapt, V_max_XRoads,
                n_steps, Steps, StepDist, ll, i,
                v_x_total=[], v_x_list=[])

            # --- identical power‑mode logic (unchanged) ----------------
            if redSpdStatus == 1 or StopFlag == 1:
                P_in, P_r, P_a, P_c, P_acc = Power_Deceleration(
                    m, step_rr, step_angle, vx, cwxA, rho, P_flat, ax_temp)
            elif reduce_lat == 1:
                ax_temp = ax_Dec_LatAcc
                P_in, P_r, P_a, P_c, P_acc = Power_Deceleration(
                    m, step_rr, step_angle, vx, cwxA, rho, P_flat, ax_temp)
            elif redSpdStatus == 2:
                ax_temp = Power_Input_Off(m, step_rr, step_angle, vx, cwxA, rho)
                P_in = P_r = P_a = P_c = P_acc = 0.0
            else:
                if step_angle <= Alpha_Vmax_steadystate:  # flat/down
                    if vx > V_max:
                        ax_temp = ax_Dec_adapt
                        P_in = P_r = P_a = P_c = P_acc = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        P_in = 0.0
                        ax_temp = Power_Input_Off(m, step_rr, step_angle, vx, cwxA, rho)
                        P_r = P_a = P_c = P_acc = 0.0
                    else:
                        ax_temp, P_in, P_r, P_a, P_c, P_acc = Power_Input_On(
                            m, step_rr, step_angle, vx, cwxA, rho, P_flat, V_max)
                else:  # climbing
                    if vx > V_max:
                        P_in = 0.0
                        ax_temp = Power_Input_Off(m, step_rr, step_angle, vx, cwxA, rho)
                        P_r = P_a = P_c = P_acc = 0.0
                    else:
                        ax_temp, P_in, P_r, P_a, P_c, P_acc = Power_Input_On(
                            m, step_rr, step_angle, vx, cwxA, rho, P_up, V_max)
            # -----------------------------------------------------------

            vx, dT, dE, dEr, dEa, dEc, dEa2 = update_velocity_and_energy(
                vx, ax_temp, step_size, P_in, P_r, P_a, P_c, P_acc)
            Time         += dT
            Energy       += dE
            Energy_roll  += dEr
            Energy_air   += dEa
            Energy_climb += dEc
            Energy_acc   += dEa2
            distance_done += step_size

    AvgSpeed = distance_done / Time if Time > 0 else 0.0
    return (Energy, Time, distance_done, AvgSpeed,
            Energy_roll, Energy_air, Energy_climb, Energy_acc)

# ======================================================================
# 3 ───────────────────────  PUBLIC API  ───────────────────────────────
# ======================================================================

def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path,
                    max_sub_step=MAX_SUB_STEP):
    """Wrapper that exposes the adaptive step size through `max_sub_step`."""
    temp = 20.0
    rho  = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926

    V_max = 10.5
    Ay_max = 2
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    (StepDist, StepElev, StepAng, StepRRcoef, _ReduceSpeedDist,
     V_lat, V_xroads) = MapData(map_file_path, 0, CrIn, Ay_max, temp)

    if len(StepDist) == 0:
        return 0.0, 0.0, 0.0, 0.0

    m = CyclistMassIn + 18.3            # bike + rider
    P_flat = CyclistPowerIn - 5.0
    P_up   = CyclistPowerIn*1.5 - 5.0

    alpha_vec, vx_vec = Free_Rolling_Slope(m, StepRRcoef[0], cwxA, rho)
    Alpha_Vmax_ss = alpha_vec[np.argmin(np.abs(np.array(vx_vec) - V_max))]

    # ---- call the adaptive simulator ----
    E, T, D, Vavg, *_ = simulate_energy(
        np.asarray(StepDist, dtype=np.float64),
        np.asarray(StepAng,  dtype=np.float64),
        np.asarray(StepRRcoef, dtype=np.float64),
        np.asarray(V_lat, dtype=np.float64),
        V_xroads,
        m, P_flat, P_up, V_max,
        ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho,
        Alpha_Vmax_ss)

    return E, T, D, Vavg

# ----------------------------------------------------------------------
if __name__ == "__main__":
    E, T, D, V = BikeEnergyModel(150.0, 70.0, 0.007, 0.45, "my_route.gpx")
    print(f"Energy: {E:.1f} J,  Time: {T:.1f} s,  Dist: {D:.1f} m,  Avg: {V*3.6:.1f} km/h")
