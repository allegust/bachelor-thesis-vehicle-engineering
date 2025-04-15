###############################################################################
# BikeEnergyOptimized.py
###############################################################################
import gpxpy
import math
import numpy as np
from numba import njit

# ---- Import each function from its own file ----
# (You must have these files in the same folder or in your PYTHONPATH).
# For example, SpeedReductionCausedByHighLatAcc.py should define the function
# `SpeedReductionCausedByHighLatAcc(...)` exactly.
from SpeedReductionCausedByHighLatAcc import Speed_Reduction_Caused_By_High_Lat_Acc
from SpeedReductionCausedByCrossRoads import Speed_Reduction_Caused_By_CrossRoads
from PowerDeceleration import Power_Deceleration
from PowerInputOff import Power_Input_Off
from PowerInputOn import Power_Input_On
from FreeRollingSlope import Free_Rolling_Slope

###############################################################################
# MapData() function (same as in your BikeEnergyOptimized.py)
###############################################################################
def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
    """
    Loads GPX data and builds stepwise route arrays matching your MATLAB approach:
     - StepDist[]: distance between each pair of GPX points
     - StepElevation[]: final elevation at each segment
     - StepAngle[]: slope angle of each segment, clipped to [-0.2, 0.2] rad
     - StepRRcoef[]: rolling resistance in each segment
     - ReduceSpeedDist[]: optional, set to 0 if no forced slowdowns
     - V_max_LatAcc[]: lateral-acc-based max speeds for cornering
     - V_max_XRoads[]: intersection-based speed limits, if any
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
    R_earth = 6371000.0

    for i in range(len(points) - 1):
        lat1, lon1, ele1 = points[i].latitude,  points[i].longitude,  points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation

        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2 +
             math.cos(math.radians(lat1))*
             math.cos(math.radians(lat2))*
             math.sin(dLon/2)**2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        dist_2d = R_earth * c
        delta_ele = ele2 - ele1
        totaldist = math.sqrt(dist_2d**2 + delta_ele**2)

        StepDist.append(totaldist)
        StepElevation.append(ele2)

        if dist_2d != 0:
            slope_angle = math.atan(delta_ele / dist_2d)
        else:
            slope_angle = 0.0

        # clip slopes to [-0.2, +0.2] rad to match the original code
        slope_angle = min(0.2, max(-0.2, slope_angle))
        StepAngle.append(slope_angle)

    # If user did not provide RRcoef_input, estimate from temperature
    if RRcoef_input == 0:
        # In your MATLAB script, you do:
        # C_r = 0.274/(temp + 46.8) + 0.004
        c_r_val = 0.274/(temp + 46.8) + 0.004
        StepRRcoef = [c_r_val] * len(StepDist)
    else:
        StepRRcoef = [RRcoef_input] * len(StepDist)

    ReduceSpeedDist = [0] * len(StepDist)

    if isinstance(Ay_max, (list, tuple)):
        chosen_ay = Ay_max[0]
    else:
        chosen_ay = Ay_max

    # Example: V_max_LatAcc ~ sqrt(a_y * big_radius). We'll keep the same placeholder approach:
    V_max_LatAcc = [math.sqrt(chosen_ay * 9999.0)] * len(StepDist)

    # For now, assume no forced cross-road slowdowns => set them to 99
    V_max_XRoads = [99]*len(StepDist)

    return (StepDist, StepElevation, StepAngle,
            StepRRcoef, ReduceSpeedDist,
            V_max_LatAcc, V_max_XRoads)

###############################################################################
# Main simulation function using Numba for speed
###############################################################################
@njit
def simulate_energy(StepDist, StepAngle, StepRRcoef, V_max_LatAcc, V_max_XRoads,
                    m, P_flat, P_up, V_max, ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho, Alpha_Vmax_steadystate):
    """
    Matches the logic in BikeEnergyModel_20241120.m:
     - For slopes <= alpha_Vmax_steadystate => use P_flat
     - For slopes > alpha_Vmax_steadystate => use P_up
    """
    Energy = 0.0
    Time = 0.0
    Energy_roll = 0.0
    Energy_air = 0.0
    Energy_climb = 0.0
    Energy_acc = 0.0

    total_steps = 0
    vx = 5.0  # same initial speed as in MATLAB

    nseg = StepDist.shape[0]
    for i in range(nseg):
        Dist_cm = int(round(StepDist[i]/0.01))
        total_steps += Dist_cm

        for _ in range(Dist_cm):
            # 1) check lateral-acc constraints
            reduce_lat = Speed_Reduction_Caused_By_High_Lat_Acc(vx, ax_Dec_LatAcc, V_max_LatAcc, i)
            # 2) check crossroad constraints
            redSpdStatus, v_x_target, ax_temp, StopFlag, SpeedReductionTable = Speed_Reduction_Caused_By_CrossRoads(
                vx, ax_Dec_adapt, V_max_XRoads, i)

            if redSpdStatus == 1 or StopFlag == 1:
                # forced deceleration
                P_in, P_roll, P_air, P_climb, P_acc = Power_Deceleration(m, StepRRcoef[i], StepAngle[i],
                                                                       vx, cwxA, rho, P_flat, ax_temp)
            elif reduce_lat == 1:
                # also deceleration by lateral-acc
                ax_temp = ax_Dec_LatAcc
                P_in, P_roll, P_air, P_climb, P_acc = Power_Deceleration(m, StepRRcoef[i], StepAngle[i],
                                                                       vx, cwxA, rho, P_flat, ax_temp)
            elif redSpdStatus == 2:
                # free-rolling => P_in=0, no acceleration
                ax_temp = Power_Input_Off(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                P_in = 0.0
                P_roll = 0.0
                P_air  = 0.0
                P_climb= 0.0
                P_acc  = 0.0

            else:
                # "normal" power mode
                # check slope vs. alpha_Vmax_steadystate => pick P_flat or P_up
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        # too fast => decelerate
                        ax_temp = ax_Dec_adapt
                        P_in = 0.0
                        P_roll = 0.0
                        P_air  = 0.0
                        P_climb= 0.0
                        P_acc  = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        # speed exactly at V_max => free roll
                        P_in = 0.0
                        ax_temp = Power_Input_Off(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                        P_roll = 0.0
                        P_air  = 0.0
                        P_climb= 0.0
                        P_acc  = 0.0
                    else:
                        # pedal with P_flat
                        ax_temp, P_in, P_roll, P_air, P_climb, P_acc = Power_Input_On(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P_flat, V_max)
                else:
                    # slope > alpha => use P_up
                    if vx > V_max:
                        P_in = 0.0
                        ax_temp = Power_Input_Off(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                        P_roll = 0.0
                        P_air  = 0.0
                        P_climb= 0.0
                        P_acc  = 0.0
                    else:
                        ax_temp, P_in, P_roll, P_air, P_climb, P_acc = Power_Input_On(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P_up, V_max)

            # update speed
            new_vx_sq = vx*vx + 2.0*ax_temp*0.01
            if new_vx_sq < 0.0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)

            if vx>0:
                dt = 0.01/vx
                Energy += P_in*dt
                Time   += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air   * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc   * dt

    Distance = total_steps*0.01
    AvgSpeed = Distance/Time if Time>0 else 0.0
    return Energy, Time, Distance, AvgSpeed, Energy_roll, Energy_air, Energy_climb, Energy_acc

###############################################################################
# Main Energy Model Function
###############################################################################
def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    Matches the logic in BikeEnergyModel_20241120.m:
      - Subtract 5 W from CyclistPowerIn => P_flat
      - Also define P_up = 1.5*CyclistPowerIn - 5 W for climbs
      - map_file_path is your .gpx file or equivalent route data
    """
    # for air density
    temp = 20.0
    rho = 101300.0 / (287.0*(temp+273.15))

    V_max = 10.5
    Ay_max = 2
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    (StepDist, StepElevation, StepAngle,
     StepRRcoef, ReduceSpeedDist,
     V_max_LatAcc, V_max_XRoads) = MapData(
        map_file_path,
        FigStatus=0,
        RRcoef_input=CrIn,
        Ay_max=Ay_max,
        temp=temp
    )

    Steps = len(StepDist)
    if (Steps != len(StepElevation)
        or Steps != len(StepRRcoef)
        or Steps != len(V_max_LatAcc)):
        print("Error: length mismatch in route arrays.")
        return 0,0,0,0

    # total system mass => cyclist + 18.3 kg
    m = CyclistMassIn + 18.3
    # P_flat => normal power on level
    P_flat = CyclistPowerIn - 5.0
    # P_up => 1.5 times CyclistPowerIn minus 5 W
    P_up   = CyclistPowerIn*1.5 - 5.0

    # Determine alpha at which free rolling speed is V_max
    #   (We import the function from separate file)
    alpha_Vector, vx_Vector = Free_Rolling_Slope(m, StepRRcoef[0], cwxA, rho)
    # find index in vx_Vector where speed ~ 10.5
    arr_vx = np.array(vx_Vector, dtype=np.float64)
    idx_vmax = np.argmin(np.abs(arr_vx - V_max))
    Alpha_Vmax_steadystate = alpha_Vector[idx_vmax]

    # Convert to arrays for numba
    StepDist_arr    = np.array(StepDist,    dtype=np.float64)
    StepAngle_arr   = np.array(StepAngle,   dtype=np.float64)
    StepRRcoef_arr  = np.array(StepRRcoef,  dtype=np.float64)
    V_max_LatAcc_arr= np.array(V_max_LatAcc,dtype=np.float64)
    V_max_XRoads_arr= np.array(V_max_XRoads,dtype=np.float64)

    # run the main loop
    (E, T, Dist, Vavg, _eroll, _eair, _eclimb, _eacc) = simulate_energy(
        StepDist_arr,
        StepAngle_arr,
        StepRRcoef_arr,
        V_max_LatAcc_arr,
        V_max_XRoads_arr,
        m,
        P_flat,
        P_up,
        V_max,
        ax_Dec_adapt,
        ax_Dec_LatAcc,
        cwxA,
        rho,
        Alpha_Vmax_steadystate
    )

    return (E, T, Dist, Vavg)

###############################################################################
# Optional test if run standalone
###############################################################################
if __name__ == "__main__":
    test_gpx_file = "my_route.gpx"  # supply your actual path
    # e.g. a cyclist with 150W, mass 70kg, Cr=0.007, cwxA=0.45
    E, T, D, V = BikeEnergyModel(150.0, 70.0, 0.007, 0.45, map_file_path=test_gpx_file)
    print("Energy:   ", E, "J")
    print("Time:     ", T, "s")
    print("Distance: ", D, "m")
    print("AvgSpeed: ", V, "m/s =>", V*3.6, "km/h")
    # Note: you can also use the BikeEnergyModel function directly in your code