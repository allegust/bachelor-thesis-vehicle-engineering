#BikeEnergyModelv3.py
import gpxpy
import math

def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
    """
    A general MapData function that:
      1) Reads a GPX file, extracting lat/lon/elev data
      2) Computes distances, slope angles, etc. for each segment
      3) Builds arrays for rolling resistance, crossroad constraints, etc.

    Inputs:
      map_file_path  - path to the GPX file to load
      FigStatus      - flag for plotting (1 or 0); not used here, but we keep the signature
      RRcoef_input   - user-chosen rolling-resistance coefficient if nonzero; else we'd add logic
      Ay_max         - maximum lateral acceleration (or array). We'll just store it as a list
      temp           - ambient temperature, used in older fallback code if RRcoef_input == 0

    Returns:
      (StepDist, StepElevation, StepAngle,
       StepRRcoef, ReduceSpeedDist, V_max_LatAcc, V_max_XRoads)
    """

    # 1) Open the GPX file
    with open(map_file_path, 'r', encoding='utf-8') as f:
        gpx_data = f.read()

    gpx = gpxpy.parse(gpx_data)

    # 2) Extract lat-lon-elev points from first track
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append(p)

    if len(points) < 2:
        print("Warning: GPX file has fewer than 2 points; no route to process.")
        return [], [], [], [], [], [], []

    # 3) Build arrays of distances, elevations, angles
    StepDist      = []
    StepElevation = []
    StepAngle     = []

    for i in range(len(points) - 1):
        lat1, lon1, ele1 = points[i].latitude,   points[i].longitude,   points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation

        # approximate horizontal distance using a spherical Earth
        R = 6371000.0
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2
             + math.cos(math.radians(lat1))
             * math.cos(math.radians(lat2))
             * math.sin(dLon/2)**2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        dist_2d = R * c

        # vertical difference
        delta_ele = ele2 - ele1
        totaldist = math.sqrt(dist_2d**2 + delta_ele**2)

        StepDist.append(totaldist)
        StepElevation.append(ele2)  # store the end elevation

        # slope = rise/run => angle in radians
        if dist_2d != 0:
            angle = math.atan(delta_ele / dist_2d)
        else:
            angle = 0.0
        # If you wish to clamp slope to ±0.2 rad (~±11.5°), you can do so here:
        if angle > 0.2:
            angle = 0.2
        elif angle < -0.2:
            angle = -0.2

        StepAngle.append(angle)

    # 4) Rolling resistance
    # If user gave a nonzero RRcoef_input, we use it. Otherwise, you'd do your formula:
    if RRcoef_input == 0:
        # Example fallback from older code:
        # C_r = 0.274/(temp + 46.8) + 0.004
        c_r_val = 0.274/(temp + 46.8) + 0.004
        StepRRcoef = [c_r_val]*len(StepDist)
    else:
        StepRRcoef = [RRcoef_input]*len(StepDist)

    # 5) Speed reductions
    # If you have logic for cross-roads, you can put it here or keep it as 0:
    ReduceSpeedDist = [0]*len(StepDist)

    # 6) Lateral-acc-based speed limit
    if isinstance(Ay_max, (list, tuple)):
        # If you have multiple cyclists, pick the middle or just the first
        chosen_ay = Ay_max[0]
    else:
        chosen_ay = Ay_max
    # We'll pick a large radius so no forced slowdown. You might want to compute curvature.
    V_max_LatAcc = [chosen_ay**0.5 * 9999**0.5]*len(StepDist)  # or just [99]*len(StepDist)
    # but let's do something simpler:
    V_max_LatAcc = [chosen_ay * 9999.0 for _ in range(len(StepDist))]

    # 7) Cross-road-based speed constraints
    V_max_XRoads = [99]*len(StepDist)

    return (StepDist, StepElevation, StepAngle,
            StepRRcoef, ReduceSpeedDist,
            V_max_LatAcc, V_max_XRoads)


def SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                                     Dist, Steps, StepDist, ll, i):
    """
    If 'vx' exceeds the segment's lateral-acc-based threshold 'V_max_LatAcc_individ[i]',
    we return 1 => reduce speed. Otherwise 0 => no need.
    """
    if vx > V_max_LatAcc_individ[i]:
        return 1
    else:
        return 0


def SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads,
                                     Dist, Steps, StepDist, ll, i,
                                     v_x_total, v_x):
    """
    If V_max_XRoads[i] < vx => decelerate => set ReduceSpeedStatus=1
    If V_max_XRoads[i] < 1 => forced stop
    """
    ReduceSpeedStatus = 0
    v_x_target = vx
    ax = 0.0
    StopFlag = 0
    SpeedReductionTable = 0

    if V_max_XRoads[i] < 1.0:
        # forced stop
        ReduceSpeedStatus = 1
        StopFlag = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 4  # e.g. 'stop sign'
    elif V_max_XRoads[i] < vx:
        # forced slowdown
        ReduceSpeedStatus = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 2  # e.g. 'yield'

    return (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable)


def PowerDeceleration(m, RRcoef, angle, vx, cwxA, rho, P, ax):
    """
    Called when cyclist is braking. 
    We treat cyclist input power as zero, and the difference is dissipated by brakes.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)

    P_roll  = F_roll * vx
    P_air   = F_air  * vx
    P_climb = F_climb*vx
    P_in    = 0.0
    P_acc   = m*ax*vx
    return (P_in, P_roll, P_air, P_climb, P_acc)


def PowerInputOff(m, RRcoef, angle, vx, cwxA, rho):
    """
    Coasting => no pedaling. 
    ax = -F_resist/m, 
    where F_resist = rolling + air + slope.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    F_resist = F_roll + F_air + F_climb
    ax = -F_resist/m
    return ax


def PowerInputOn(m, RRcoef, angle, vx, cwxA, rho, P, V_max):
    """
    Cyclist is providing power = P. We subtract rolling/air/climb power to see leftover for acc.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    P_roll  = F_roll * vx
    P_air   = F_air  * vx
    P_climb = F_climb*vx

    ResistPower = P_roll + P_air + P_climb
    leftover = P - ResistPower
    if leftover < 0:
        P_acc = leftover if vx>0 else 0.0
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P
    else:
        P_acc = leftover
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P

    return (ax, P_in, P_roll, P_air, P_climb, P_acc)


def FreeRollingSlope(m, RRcoef, cwxA, rho):
    """
    For speeds from 0.5..20 m/s, compute slope angle alpha for free rolling (net force = 0).
    alpha = asin( (F_roll+F_air)/(m*g) ).
    """
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


def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    Translated from Matlab function:
      [Energy, Time, Distance, AvgSpeed] = BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA)

    Now requires an extra param: map_file_path (the GPX route).
    """
    # For the density calculation, we replicate your existing formula:
    temp = 20.0
    rho  = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926

    V_max         = 10.5
    Ay_max        = 2
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    # 1) Retrieve route data from the MapData function
    (StepDist, StepElevation, StepAngle, StepRRcoef,
     ReduceSpeedDist, V_max_LatAcc, V_max_XRoads) = MapData(
        map_file_path, FigStatus=0, RRcoef_input=CrIn, Ay_max=Ay_max, temp=temp
    )

    Steps = len(StepDist)
    if Steps != len(StepElevation):
        print('Error: length mismatch in StepElevation array.')
    elif Steps != len(StepRRcoef):
        print('Error: length mismatch in StepRRcoef array.')
    elif Steps != len(V_max_LatAcc):
        print('Error: length mismatch in V_max_LatAcc array.')

    # 2) Prepare cyclist data
    m = CyclistMassIn + 18.3
    P = CyclistPowerIn - 5
    P_up = CyclistPowerIn*1.5 - 5
    V_max_LatAcc_individ = V_max_LatAcc

    # 3) Precompute free-rolling slopes for referencing
    alpha_Vector, vx_Vector = FreeRollingSlope(m, StepRRcoef[0], cwxA, rho)
    try:
        idx_vmax = vx_Vector.index(V_max)
        Alpha_Vmax_steadystate = alpha_Vector[idx_vmax]
    except ValueError:
        print(f"Warning: V_max={V_max} not found in vx_Vector. Using alpha=0.0")
        Alpha_Vmax_steadystate = 0.0

    # 4) Time/Energy accumulators
    Time = 0.0
    Energy = 0.0
    Energy_roll  = 0.0
    Energy_air   = 0.0
    Energy_climb = 0.0
    Energy_acc   = 0.0

    # We'll keep a speed array so that you can see the final speed at each substep
    v_x_total = []
    vx = 5.0    # start speed
    StopFlag = 0
    ax = 0.0

    # 5) Main loop over segments
    for i in range(Steps):
        Dist_cm = round(StepDist[i]/0.01)  # number of 1-cm steps
        for ll in range(Dist_cm):
            # 1) check if we must slow for lateral acceleration
            ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(
                vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                Dist_cm, Steps, StepDist, ll, i
            )

            # 2) check crossroad-based slowdown
            (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable) = \
                SpeedReductionCausedByCrossRoads(
                    vx, ax_Dec_adapt, V_max_XRoads,
                    Dist_cm, Steps, StepDist, ll, i,
                    v_x_total, vx
                )

            # default partial powers
            P_in    = 0.0
            P_roll  = 0.0
            P_air   = 0.0
            P_climb = 0.0
            P_acc   = 0.0

            if (ReduceSpeedStatus == 1 or StopFlag == 1):
                # forced deceleration
                P_in, P_roll, P_air, P_climb, P_acc = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedFlag == 1:
                # lat-acc deceleration
                ax = ax_Dec_LatAcc
                P_in, P_roll, P_air, P_climb, P_acc = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedStatus == 2:
                # free rolling
                ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
            else:
                # normal slope-based logic
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        # decelerate to V_max
                        ax = ax_Dec_adapt
                        P_in = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        # coasting at V_max
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        ax, P_in, P_roll, P_air, P_climb, P_acc = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )
                else:
                    if vx > V_max:
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        ax, P_in, P_roll, P_air, P_climb, P_acc = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )

            # 6) update speed from v^2 + 2 a s
            new_vx_sq = vx**2 + 2.0*ax*0.01
            if new_vx_sq < 0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)

            # if we've braked to near zero and had a stop sign, reset it
            if vx < 0.01 and StopFlag == 1:
                StopFlag = 0

            # 7) accumulate
            if vx > 0:
                dt = 0.01/vx  # time for each 1-cm chunk
                Energy     += P_in    * dt
                Time       += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air   * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc   * dt

        # store final speed of this segment repeated Dist_cm times
        v_x_total.extend([vx]*Dist_cm)

    Distance = len(v_x_total)/100.0  # in meters
    AvgSpeed = Distance/Time if Time>0 else 0.0

    return (Energy, Time, Distance, AvgSpeed)


# If you want to test standalone:
#if __name__ == "__main__":
    # Simple test call with dummy inputs:
    # (Give a real GPX path here if you want to see it run)
    test_gpx_file = "path/to/some_route.gpx"
    E, T, D, V = BikeEnergyModel(150, 70, 0.007, 0.45, map_file_path=test_gpx_file)
    print("Energy:", E, "J")
    print("Time:", T, "s")
    print("Distance:", D, "m")
    print("AvgSpeed:", V, "m/s =>", V*3.6, "km/h")
