# BikeEnergyModel.py
import gpxpy
import math
import numpy as np
from numba import njit

def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
    """
    A general MapData function that:
      1) Reads a GPX file, extracting lat/lon/elev data
      2) Computes distances, slope angles, etc. for each segment
      3) Builds arrays for rolling resistance, crossroad constraints, etc.
    """
    with open(map_file_path, 'r', encoding='utf-8') as f:
        gpx_data = f.read()

    gpx = gpxpy.parse(gpx_data)
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append(p)

    if len(points) < 2:
        print("Warning: GPX file has fewer than 2 points; no route to process.")
        return [], [], [], [], [], [], []

    StepDist = []
    StepElevation = []
    StepAngle = []
    for i in range(len(points) - 1):
        lat1, lon1, ele1 = points[i].latitude, points[i].longitude, points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation
        R = 6371000.0
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2 +
             math.cos(math.radians(lat1)) *
             math.cos(math.radians(lat2)) *
             math.sin(dLon/2)**2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        dist_2d = R * c
        delta_ele = ele2 - ele1
        totaldist = math.sqrt(dist_2d**2 + delta_ele**2)
        StepDist.append(totaldist)
        StepElevation.append(ele2)
        if dist_2d != 0:
            angle = math.atan(delta_ele / dist_2d)
        else:
            angle = 0.0
        if angle > 0.2:
            angle = 0.2
        elif angle < -0.2:
            angle = -0.2
        StepAngle.append(angle)
        
    if RRcoef_input == 0:
        c_r_val = 0.274/(temp + 46.8) + 0.004
        StepRRcoef = [c_r_val] * len(StepDist)
    else:
        StepRRcoef = [RRcoef_input] * len(StepDist)

    ReduceSpeedDist = [0] * len(StepDist)
    if isinstance(Ay_max, (list, tuple)):
        chosen_ay = Ay_max[0]
    else:
        chosen_ay = Ay_max
    V_max_LatAcc = [chosen_ay * 9999.0 for _ in range(len(StepDist))]
    V_max_XRoads = [99] * len(StepDist)

    return (StepDist, StepElevation, StepAngle,
            StepRRcoef, ReduceSpeedDist,
            V_max_LatAcc, V_max_XRoads)

# ---------------------- Optimized helper functions using Numba ----------------------

@njit
def SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc_individ, i):
    # Only the current segment value is needed
    if vx > V_max_LatAcc_individ[i]:
        return 1
    else:
        return 0

@njit
def SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads, i):
    ReduceSpeedStatus = 0
    v_x_target = vx
    ax = 0.0
    StopFlag = 0
    SpeedReductionTable = 0
    if V_max_XRoads[i] < 1.0:
        ReduceSpeedStatus = 1
        StopFlag = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 4
    elif V_max_XRoads[i] < vx:
        ReduceSpeedStatus = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 2
    return ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable

@njit
def PowerDeceleration(m, RRcoef, angle, vx, cwxA, rho, P, ax):
    F_roll  = m * 9.81 * math.cos(angle) * RRcoef
    F_air   = 0.5 * cwxA * rho * (vx**2)
    F_climb = m * 9.81 * math.sin(angle)
    P_roll  = F_roll * vx
    P_air   = F_air * vx
    P_climb = F_climb * vx
    P_in    = 0.0
    P_acc   = m * ax * vx
    return P_in, P_roll, P_air, P_climb, P_acc

@njit
def PowerInputOff(m, RRcoef, angle, vx, cwxA, rho):
    F_roll  = m * 9.81 * math.cos(angle) * RRcoef
    F_air   = 0.5 * cwxA * rho * (vx**2)
    F_climb = m * 9.81 * math.sin(angle)
    F_resist = F_roll + F_air + F_climb
    ax = -F_resist / m
    return ax

@njit
def PowerInputOn(m, RRcoef, angle, vx, cwxA, rho, P, V_max):
    F_roll  = m * 9.81 * math.cos(angle) * RRcoef
    F_air   = 0.5 * cwxA * rho * (vx**2)
    F_climb = m * 9.81 * math.sin(angle)
    P_roll  = F_roll * vx
    P_air   = F_air * vx
    P_climb = F_climb * vx
    ResistPower = P_roll + P_air + P_climb
    leftover = P - ResistPower
    if leftover < 0:
        P_acc = leftover if vx > 0 else 0.0
        ax = (P_acc / vx) / m if vx > 0 else 0.0
        P_in = P
    else:
        P_acc = leftover
        ax = (P_acc / vx) / m if vx > 0 else 0.0
        P_in = P
    return ax, P_in, P_roll, P_air, P_climb, P_acc

# ---------------------- Main simulation function ----------------------

@njit
def simulate_energy(StepDist, StepAngle, StepRRcoef, V_max_LatAcc, V_max_XRoads,
                    m, P, V_max, ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho, Alpha_Vmax_steadystate):
    Energy = 0.0
    Time = 0.0
    Energy_roll = 0.0
    Energy_air = 0.0
    Energy_climb = 0.0
    Energy_acc = 0.0
    total_steps = 0  # Count total 1-cm steps
    vx = 5.0  # Starting speed in m/s
    # Loop over segments (each segment corresponds to a route between two GPX points)
    nseg = StepDist.shape[0]
    for i in range(nseg):
        # Determine number of 1-cm steps for the current segment
        Dist_cm = int(round(StepDist[i] / 0.01))
        total_steps += Dist_cm
        for ll in range(Dist_cm):
            # Check for lateral acceleration constraint
            ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc, i)
            # Check for crossroads-based deceleration
            ReduceSpeedStatus, v_x_target, ax_temp, StopFlag, SpeedReductionTable = SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads, i)
            # Choose the power mode
            if ReduceSpeedStatus == 1 or StopFlag == 1:
                P_in, P_roll, P_air, P_climb, P_acc = PowerDeceleration(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax_temp)
            elif ReduceSpeedFlag == 1:
                ax_temp = ax_Dec_LatAcc
                P_in, P_roll, P_air, P_climb, P_acc = PowerDeceleration(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax_temp)
            elif ReduceSpeedStatus == 2:
                ax_temp = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                P_in = 0.0
                P_roll = 0.0
                P_air = 0.0
                P_climb = 0.0
                P_acc = 0.0
            else:
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        ax_temp = ax_Dec_adapt
                        P_in = 0.0
                        P_roll = 0.0
                        P_air = 0.0
                        P_climb = 0.0
                        P_acc = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        P_in = 0.0
                        ax_temp = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                        P_roll = 0.0
                        P_air = 0.0
                        P_climb = 0.0
                        P_acc = 0.0
                    else:
                        ax_temp, P_in, P_roll, P_air, P_climb, P_acc = PowerInputOn(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max)
                else:
                    if vx > V_max:
                        P_in = 0.0
                        ax_temp = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                        P_roll = 0.0
                        P_air = 0.0
                        P_climb = 0.0
                        P_acc = 0.0
                    else:
                        ax_temp, P_in, P_roll, P_air, P_climb, P_acc = PowerInputOn(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max)
            # Update the cyclist's speed using v^2 + 2*a*s with s = 0.01 m
            new_vx_sq = vx * vx + 2.0 * ax_temp * 0.01
            if new_vx_sq < 0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)
            # Accumulate time and energy if moving
            if vx > 0:
                dt = 0.01 / vx
                Energy += P_in * dt
                Time += dt
                Energy_roll += P_roll * dt
                Energy_air += P_air * dt
                Energy_climb += P_climb * dt
                Energy_acc += P_acc * dt
    Distance = total_steps * 0.01  # Total distance in meters
    AvgSpeed = Distance / Time if Time > 0 else 0.0
    return Energy, Time, Distance, AvgSpeed, Energy_roll, Energy_air, Energy_climb, Energy_acc

# ---------------------- Main Energy Model Function ----------------------

def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    Translated from Matlab:
      [Energy, Time, Distance, AvgSpeed] = BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA)
    Now takes an extra parameter map_file_path (the GPX route).
    """
    temp = 20.0
    rho  = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926
    V_max         = 10.5
    Ay_max        = 2
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    (StepDist, StepElevation, StepAngle,
     StepRRcoef, ReduceSpeedDist,
     V_max_LatAcc, V_max_XRoads) = MapData(
        map_file_path, FigStatus=0, RRcoef_input=CrIn, Ay_max=Ay_max, temp=temp
    )

    Steps = len(StepDist)
    if Steps != len(StepElevation) or Steps != len(StepRRcoef) or Steps != len(V_max_LatAcc):
        print('Error: length mismatch in route arrays.')
        return 0, 0, 0, 0

    m = CyclistMassIn + 18.3
    P = CyclistPowerIn - 5

    # Precompute free-rolling slopes using the first segment's rolling resistance as a proxy
    alpha_Vector, vx_Vector = FreeRollingSlope(m, StepRRcoef[0], cwxA, rho)
    try:
        idx_vmax = np.argmin(np.abs(np.array(vx_Vector) - V_max))
        Alpha_Vmax_steadystate = alpha_Vector[idx_vmax]
    except ValueError:
        print(f"Warning: V_max={V_max} not found in vx_Vector. Using alpha=0.0")
        Alpha_Vmax_steadystate = 0.0

    # Convert route lists to NumPy arrays for efficient JIT simulation
    StepDist_arr = np.array(StepDist, dtype=np.float64)
    StepAngle_arr = np.array(StepAngle, dtype=np.float64)
    StepRRcoef_arr = np.array(StepRRcoef, dtype=np.float64)
    V_max_LatAcc_arr = np.array(V_max_LatAcc, dtype=np.float64)
    V_max_XRoads_arr = np.array(V_max_XRoads, dtype=np.float64)

    # Run the simulation (the heavy inner loops are now JIT compiled)
    Energy, Time, Distance, AvgSpeed, Energy_roll, Energy_air, Energy_climb, Energy_acc = simulate_energy(
        StepDist_arr, StepAngle_arr, StepRRcoef_arr, V_max_LatAcc_arr, V_max_XRoads_arr,
        m, P, V_max, ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho, Alpha_Vmax_steadystate
    )

    return (Energy, Time, Distance, AvgSpeed)

# ---------------------- FreeRollingSlope Function (as in original) ----------------------

def FreeRollingSlope(m, RRcoef, cwxA, rho):
    vx_Vector = [v*0.5 for v in range(1, 41)]
    alpha_Vector = []
    for vx in vx_Vector:
        F_roll = m*9.81*RRcoef
        F_air  = 0.5*cwxA*rho*(vx**2)
        top = (F_roll + F_air)/(m*9.81)
        if top > 1.0:
            alpha = math.pi/2.0
        elif top < -1.0:
            alpha = -math.pi/2.0
        else:
            alpha = math.asin(top)
        alpha_Vector.append(alpha)
    return alpha_Vector, vx_Vector

# ---------------------- Standalone Test ----------------------
#if __name__ == "__main__":
    test_gpx_file = "path/to/some_route.gpx"
    E, T, D, V = BikeEnergyModel(150, 70, 0.007, 0.45, map_file_path=test_gpx_file)
    print("Energy:", E, "J")
    print("Time:", T, "s")
    print("Distance:", D, "m")
    print("AvgSpeed:", V, "m/s =>", V*3.6, "km/h")
