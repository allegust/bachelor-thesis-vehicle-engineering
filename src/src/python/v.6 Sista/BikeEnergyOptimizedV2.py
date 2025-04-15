###############################################################################
# BikeEnergyOptimizedV2.py
###############################################################################
import gpxpy
import math
import numpy as np

# Import each function from its separate files, as in your MATLAB approach:
from SpeedReductionCausedByHighLatAcc import Speed_Reduction_Caused_By_High_Lat_Acc
from SpeedReductionCausedByCrossRoads import Speed_Reduction_Caused_By_CrossRoads
from PowerDeceleration import Power_Deceleration
from PowerInputOff import Power_Input_Off
from PowerInputOn import Power_Input_On
from FreeRollingSlope import Free_Rolling_Slope


def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
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
        lat1, lon1, ele1 = (
            points[i].latitude,
            points[i].longitude,
            points[i].elevation
        )
        lat2, lon2, ele2 = (
            points[i+1].latitude,
            points[i+1].longitude,
            points[i+1].elevation
        )

        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2 +
             math.cos(math.radians(lat1)) *
             math.cos(math.radians(lat2)) *
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

        # clip slope to [-0.2, +0.2]
        slope_angle = max(-0.2, min(0.2, slope_angle))
        StepAngle.append(slope_angle)

    # Rolling resistance
    if RRcoef_input == 0:
        c_r_val = 0.274/(temp + 46.8) + 0.004
        StepRRcoef = [c_r_val]*len(StepDist)
    else:
        StepRRcoef = [RRcoef_input]*len(StepDist)

    ReduceSpeedDist = [0]*len(StepDist)

    # for lateral-acc
    if isinstance(Ay_max, (list, tuple)):
        chosen_ay = Ay_max[0]
    else:
        chosen_ay = Ay_max

    V_max_LatAcc = [math.sqrt(chosen_ay*9999.0)]*len(StepDist)

# ------------------------------------------------------------------
    # Build V_max_XRoads as Nx2 => (cumulativeDistance, status)
    # where status is one of 1,2,3,4, matching the MATLAB logic:
    #   1 => free rolling
    #   2 => brake to 4 m/s
    #   3 => brake to 2 m/s
    #   4 => stop
    # (You can adapt the distances or statuses as needed.)
    # ------------------------------------------------------------------
    V_max_XRoads = []
    dist_accum = 0.0
    for dist in StepDist:
        dist_accum += dist
        status = 1  # means “no forced slowdown”
        V_max_XRoads.append([dist_accum, status])

    # convert to float array
    V_max_XRoads = np.array(V_max_XRoads, dtype=float)

    return (
        StepDist,
        StepElevation,
        StepAngle,
        StepRRcoef,
        ReduceSpeedDist,
        V_max_LatAcc,
        V_max_XRoads
    )

""" OLD                                                                                 OLD
    # ------------------------------------------------------------------
    # NEW: Build V_max_XRoads as Nx2 (distance, slowdownStatus).
    #   We'll just store the cumulative distance and a dummy "1" => free rolling.
    # ------------------------------------------------------------------
    V_max_XRoads = []
    dist_accum = 0.0
    for i, d in enumerate(StepDist):
        dist_accum += d
        # For example, give everything a speed-reduction "level 1" = free-rolling
        # (You can put real data for each segment.)
        V_max_XRoads.append([dist_accum, 1])  # distance so far, status=1

    V_max_XRoads = np.array(V_max_XRoads, dtype=float)
    return (
        StepDist,
        StepElevation,
        StepAngle,
        StepRRcoef,
        ReduceSpeedDist,
        V_max_LatAcc,
        V_max_XRoads
    )
"""


def simulate_energy(StepDist, StepAngle, StepRRcoef,
                    V_max_LatAcc, V_max_XRoads,
                    m, P_flat, P_up, V_max,
                    ax_Dec_adapt, ax_Dec_LatAcc, cwxA, rho,
                    Alpha_Vmax_steadystate):
    """
    Imitates BikeEnergyModel_20241120.m logic with nested loops:
      - Outer loop over Steps = number of map segments
      - Inner loop over Dist = 1 cm increments
      - Keep a v_x_total array that accumulates velocities (like in MATLAB).
    """
    Steps = len(StepDist)
    Time = 0.0
    Energy = 0.0
    Energy_roll = 0.0
    Energy_air = 0.0
    Energy_climb = 0.0
    Energy_acc = 0.0

    vx = 5.0  # initial speed in m/s (like MATLAB code's default)
    v_x_total = []  # store velocities across all segments and cm-steps

    total_cm = 0  # total steps in cm to measure final distance

    for i in range(Steps):
        step_size = 0.5  # new step size: 10 cm
        # # of 1cm increments for step i
        Dist_cm = int(round(StepDist[i] / step_size))                                     #0.01 ska vara
        total_cm += Dist_cm

        # for MATLAB, we keep a local array v_x for each step if you want
        # but eventually we do e.g. v_x_total = [v_x_total, v_x].
        # We'll do it similarly:
        v_x_segment = []

        for ll in range(Dist_cm):

            # Check lat-acc constraints
            reduce_lat = Speed_Reduction_Caused_By_High_Lat_Acc(
                vx, ax_Dec_LatAcc, V_max_LatAcc,
                Dist_cm, Steps, StepDist, ll, i
            )
            # Crossroad constraints (pass in v_x_total as a list, plus current vx)
            # to replicate "Dist_now = (length(v_x_total) + length(v_x)) / 100"
            # after we do the new velocity at the end
            partial_v_x = v_x_total + v_x_segment  # all velocities so far
            redSpdStatus, v_x_target, ax_temp, StopFlag, SpeedReductionTable = Speed_Reduction_Caused_By_CrossRoads(
                vx, ax_Dec_adapt, V_max_XRoads,
                Dist_cm, Steps, StepDist, ll, i,
                v_x_total=partial_v_x,  # pass a list for v_x_total
                v_x_list=partial_v_x    # pass a list for v_x
            )

            # Now the main logic for power
            if redSpdStatus == 1 or StopFlag == 1:
                P_in, P_roll, P_air_, P_climb, P_acc_ = Power_Deceleration(
                    m, StepRRcoef[i], StepAngle[i],
                    vx, cwxA, rho, P_flat, ax_temp
                )
            elif reduce_lat == 1:
                ax_temp = ax_Dec_LatAcc
                P_in, P_roll, P_air_, P_climb, P_acc_ = Power_Deceleration(
                    m, StepRRcoef[i], StepAngle[i],
                    vx, cwxA, rho, P_flat, ax_temp
                )
            elif redSpdStatus == 2:
                # free rolling
                ax_temp = Power_Input_Off(m, StepRRcoef[i], StepAngle[i],
                                          vx, cwxA, rho)
                P_in = 0.0
                P_roll = 0.0
                P_air_ = 0.0
                P_climb = 0.0
                P_acc_  = 0.0
            else:
                # normal power mode
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        ax_temp = ax_Dec_adapt
                        P_in = 0.0
                        P_roll = 0.0
                        P_air_ = 0.0
                        P_climb= 0.0
                        P_acc_ = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        P_in = 0.0
                        ax_temp = Power_Input_Off(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho
                        )
                        P_roll = 0.0
                        P_air_ = 0.0
                        P_climb= 0.0
                        P_acc_ = 0.0
                    else:
                        ax_temp, P_in, P_roll, P_air_, P_climb, P_acc_ = Power_Input_On(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho,
                            P_flat, V_max
                        )
                else:
                    # slope > alpha => use P_up
                    if vx > V_max:
                        P_in = 0.0
                        ax_temp = Power_Input_Off(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho
                        )
                        P_roll = 0.0
                        P_air_ = 0.0
                        P_climb= 0.0
                        P_acc_ = 0.0
                    else:
                        ax_temp, P_in, P_roll, P_air_, P_climb, P_acc_ = Power_Input_On(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho,
                            P_up, V_max
                        )

            # update speed
            new_vx_sq = vx*vx + 2.0*ax_temp*step_size                                    # new_vx_sq = vx*vx + 2.0*ax_temp*0.01
            if new_vx_sq < 0.0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)

            # time & energy
            if vx > 0.0:
                dt = step_size/vx                                                            # dt = 0.01/vx
                Energy += P_in * dt
                Time   += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air_  * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc_  * dt

            # store the updated velocity in this step's array
            v_x_segment.append(vx)

        # after finishing the Dist_cm loop, add the segment's velocities to the global v_x_total
        v_x_total.extend(v_x_segment)

    # final distance is total_cm * 0.01
    Distance = total_cm * step_size  # convert to meters                                     # Distance = total_cm * 0.01
    AvgSpeed = Distance / Time if Time>0 else 0.0

    return (Energy, Time, Distance, AvgSpeed,
            Energy_roll, Energy_air, Energy_climb, Energy_acc)


def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    Matches the logic of BikeEnergyModel_20241120.m:
      - Subtract 5 W from CyclistPowerIn => P_flat
      - 1.5 * CyclistPowerIn - 5 W => P_up
      - MapData -> StepDist, ...
      - simulate_energy -> does the nested loops
    """
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
    if (Steps==0 or Steps!=len(StepElevation)
        or Steps!=len(StepRRcoef)
        or Steps!=len(V_max_LatAcc)):
        print("Error: length mismatch or empty route data.")
        return (0.0, 0.0, 0.0, 0.0)

    # total mass
    m = CyclistMassIn + 18.3

    # define powers
    P_flat = CyclistPowerIn - 5.0
    P_up   = CyclistPowerIn*1.5 - 5.0

    # free rolling slope angle for V_max
    alpha_Vector, vx_Vector = Free_Rolling_Slope(m, StepRRcoef[0], cwxA, rho)
    arr_vx = np.array(vx_Vector, dtype=np.float64)
    idx_vmax = np.argmin(np.abs(arr_vx - V_max))
    Alpha_Vmax_steadystate = alpha_Vector[idx_vmax]

    # convert StepDist etc. to arrays if you want, but we can just keep them as lists
    StepDist_arr    = np.array(StepDist,    dtype=np.float64)
    StepAngle_arr   = np.array(StepAngle,   dtype=np.float64)
    StepRRcoef_arr  = np.array(StepRRcoef,  dtype=np.float64)
    V_max_LatAcc_arr= np.array(V_max_LatAcc,dtype=np.float64)
    V_max_XRoads_arr= np.array(V_max_XRoads,dtype=np.float64)

    E, T, Dist, Vavg, _, _, _, _ = simulate_energy(
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


# Quick test
if __name__ == "__main__":
    test_gpx_file = "my_route.gpx"
    E, T, D, V = BikeEnergyModel(150.0, 70.0, 0.007, 0.45, map_file_path=test_gpx_file)
    print("Energy: ", E, "J")
    print("Time:   ", T, "s")
    print("Dist:   ", D, "m")
    print("AvgSpd: ", V, "m/s =>", V*3.6, "km/h")
