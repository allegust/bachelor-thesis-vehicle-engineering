import gpxpy
import gpxpy.gpx
import math

def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
    """
    Reads a GPX file, computes stepwise distance, elevation, angle,
    and sets speed-limiting arrays.

    Note that in the MATLAB code, different 'MapData_...' M-files exist
    (e.g., MapData_DD_Koenigsbruecker_down). For a truly identical workflow,
    you would replicate that specialized logic here. For general usage,
    we parse the GPX and assume no forced stops or special speed limits.
    """
    with open(map_file_path, 'r', encoding='utf-8') as f:
        gpx = gpxpy.parse(f)

    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append(p)

    StepDist = []
    StepElevation = []
    StepAngle = []
    R = 6371000  # Earth radius
    for i in range(len(points) - 1):
        lat1, lon1, ele1 = points[i].latitude,   points[i].longitude,   points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation

        # 2D haversine
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat / 2)**2 +
             math.cos(math.radians(lat1)) *
             math.cos(math.radians(lat2)) *
             math.sin(dLon / 2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist_2d = R * c

        delta_ele = ele2 - ele1
        totaldist = math.sqrt(dist_2d**2 + delta_ele**2)
        StepDist.append(totaldist)
        StepElevation.append(ele2)

        slope = (delta_ele / dist_2d) if dist_2d != 0 else 0.0
        angle = math.atan(slope)
        StepAngle.append(angle)

    # Rolling resistance coefficient array
    StepRRcoef = [RRcoef_input]*len(StepDist)

    # For demonstration: no forced slowdowns or stops
    ReduceSpeedDist = [0]*len(StepDist)  # 0 => no speed reduction
    V_max_LatAcc    = [Ay_max]*len(StepDist)
    V_max_XRoads    = [99]*len(StepDist) # 99 => no forced speed limit from cross-roads

    return (StepDist, StepElevation, StepAngle,
            StepRRcoef, ReduceSpeedDist, V_max_LatAcc, V_max_XRoads)


def SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                                     Dist, Steps, StepDist, ll, i):
    """
    Same logic as in the MATLAB code:
      If current speed is above the lateral-acc-based threshold, return 1 => we must decelerate.
    """
    if vx > V_max_LatAcc_individ[i]:
        return 1
    else:
        return 0


def SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads,
                                     Dist, Steps, StepDist, ll, i,
                                     v_x_total, v_x):
    """
    Returns a tuple of (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable).
    For this generic version, if V_max_XRoads[i] < 1 => we must come to a stop (StopFlag=1).
    If 1 <= V_max_XRoads[i] < vx => we must slow down => SpeedReductionStatus=1 or 2, etc.

    Adjust or expand as needed to replicate your own intersection logic.
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
        SpeedReductionTable = 4  # e.g., "stop sign"
    elif V_max_XRoads[i] < vx:
        # forced slowdown
        ReduceSpeedStatus = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 2

    return (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable)


def PowerDeceleration(m, RRcoef, angle, vx, cwxA, rho, P, ax):
    """
    Same approach as the MATLAB code's deceleration branch:
      - cyclist input power = 0
      - partial powers assigned to roll, air, climb, acc
      - negative P_acc means the brakes do the job.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)

    # partial powers:
    P_roll  = F_roll * vx
    P_air   = F_air  * vx
    P_climb = F_climb * vx
    # cyclist is not pedaling => P_in=0
    P_in = 0.0
    # mechanical acceleration power
    P_acc = m * ax * vx  # negative if slowing down

    return (P_in, P_roll, P_air, P_climb, P_acc)


def PowerInputOff(m, RRcoef, angle, vx, cwxA, rho):
    """
    Same as the free-rolling branch in MATLAB: no pedaling power,
    so net acceleration depends on the sum of rolling, air, climb.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    F_resist = F_roll + F_air + F_climb
    ax = -F_resist / m
    return ax


def PowerInputOn(m, RRcoef, angle, vx, cwxA, rho, P, V_max):
    """
    Cyclist is pedaling with input power P. Compare with sum of rolling,
    air, and climb power. leftover = P - (sum of those). If leftover<0,
    then the cyclist can't maintain that speed, so ax is negative.

    The logic matches the MATLAB code's 'PowerInputOn' portion.
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)

    P_roll  = F_roll*vx
    P_air   = F_air*vx
    P_climb = F_climb*vx

    ResistPower = P_roll + P_air + P_climb
    leftover = P - ResistPower
    if leftover < 0:
        # can't maintain speed => negative acceleration
        P_acc = leftover
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P
    else:
        # leftover power used for positive acceleration
        P_acc = leftover
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P

    return (ax, P_in, P_roll, P_air, P_climb, P_acc)


def FreeRollingSlope(m, RRcoef, cwxA, rho):
    """
    Compute the angle alpha at which the net force is zero for speeds
    from 0.5 m/s up to 20 m/s. This is used in the MATLAB code to check
    whether a given slope is 'steep enough' that the cyclist is satisfied
    with free-rolling, or must brake, etc.
    """
    vx_Vector = [v*0.5 for v in range(1, 41)]  # 0.5 .. 20 m/s
    alpha_Vector = []
    for vx in vx_Vector:
        F_roll = m*9.81*RRcoef
        F_air  = 0.5*cwxA*rho*(vx**2)
        # if net=0 => m*g*sin(alpha)=F_roll+F_air => sin(alpha)=...
        sum_forces = F_roll + F_air
        top = sum_forces/(m*9.81)
        if top>1.0:
            alpha = math.pi/2
        elif top<-1.0:
            alpha = -math.pi/2
        else:
            alpha = math.asin(top)
        alpha_Vector.append(alpha)
    return alpha_Vector, vx_Vector


def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    This function aims to replicate the loop-by-loop logic of
    BikeEnergyModel_20241120.m (MATLAB), step by step.

    Returns:
        Energy (J),
        Time (s),
        Distance (m),
        AvgSpeed (m/s)
    """
    # constants
    temp = 20.0
    rho = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926

    V_max = 10.5        # [m/s]
    Ay_max = 2          # [m/s^2]
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    # 1) load map data
    (StepDist, StepElevation, StepAngle,
     StepRRcoef, ReduceSpeedDist, V_max_LatAcc, V_max_XRoads) = MapData(
        map_file_path, 0, CrIn, Ay_max, temp
    )

    Steps = len(StepDist)
    if (Steps != len(StepElevation)
        or Steps != len(StepRRcoef)
        or Steps != len(V_max_LatAcc)):
        print(" Error in MapData function: length mismatch.")
    
    # System mass, power
    m = CyclistMassIn + 18.3
    # match the MATLAB line:  "P = CyclistPowerIn - 5;"
    P = CyclistPowerIn - 5

    # "P_up = CyclistPowerIn * 1.5 - 5" is defined but not used in MATLAB
    # We'll keep it for completeness.
    P_up = CyclistPowerIn * 1.5 - 5

    # compute free rolling slope limit
    alpha_Vector, vx_Vector = FreeRollingSlope(m, StepRRcoef[0], cwxA, rho)
    try:
        V_max_Index = vx_Vector.index(V_max)
        Alpha_Vmax_steadystate = alpha_Vector[V_max_Index]
    except ValueError:
        print("Warning: V_max not found exactly in vx_Vector; using alpha=0.")
        Alpha_Vmax_steadystate = 0.0

    # accumulators
    Time = 0.0
    Energy = 0.0
    Energy_roll  = 0.0
    Energy_air   = 0.0
    Energy_climb = 0.0
    Energy_acc   = 0.0

    v_x_total = []
    vx = 5.0            # initial speed, in MATLAB set to 5
    StopFlag = 0
    ax = 0.0

    for i in range(Steps):
        # same subdividing approach: Dist = round(StepDist(i)/0.01)
        Dist = int(round(StepDist[i]/0.01))
        if Dist<1:
            Dist = 1
        for ll in range(Dist):
            # check whether we must brake for lateral acc
            ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(
                vx, ax_Dec_LatAcc, V_max_LatAcc, Dist, Steps, StepDist, ll, i
            )
            (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable) = \
                SpeedReductionCausedByCrossRoads(
                    vx, ax_Dec_adapt, V_max_XRoads,
                    Dist, Steps, StepDist, ll, i, v_x_total, vx
                )

            P_roll = 0.0
            P_air  = 0.0
            P_climb= 0.0
            P_acc  = 0.0
            P_in   = 0.0

            if (ReduceSpeedStatus == 1 or StopFlag == 1):
                # deceleration
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedFlag == 1:
                # deceleration with latAcc
                ax = ax_Dec_LatAcc
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedStatus == 2:
                # free rolling e.g. because of yield
                P_in = 0.0
                ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
            else:
                # slope check vs alpha_Vmax_steadystate
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        # too fast => dec adapt
                        ax = ax_Dec_adapt
                        P_in = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        # exactly on V_max => free rolling
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i],
                                           vx, cwxA, rho)
                    else:
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )
                else:
                    # slope bigger than alpha => if vx>V_max => free rolling
                    if vx > V_max:
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i],
                                           vx, cwxA, rho)
                    else:
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )

            # --- Clamp negative input power to zero to match the MATLAB code ---
            if P_in < 0.0:
                P_in = 0.0

            # update vx via v^2 + 2 a s
            new_vx_sq = vx**2 + 2.0*ax*0.01
            if new_vx_sq < 0.0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)

            if vx<0.01 and StopFlag == 1:
                StopFlag = 0

            # accumulate energy/time only if vx>0
            if vx>0:
                dt = 0.01/vx
                Energy += P_in * dt
                Time   += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air   * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc   * dt

        # store final speed of each segment
        v_x_total.append(vx)

    Distance = len(v_x_total)*0.01
    AvgSpeed = Distance/Time if Time>0 else 0

    return (Energy, Time, Distance, AvgSpeed)
