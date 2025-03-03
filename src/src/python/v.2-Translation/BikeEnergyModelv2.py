#BikeEnergyModelv2.py
import gpxpy
import gpxpy.gpx
import math

def MapData(map_file_path, FigStatus, RRcoef_input, Ay_max, temp):
    # 1) Open the GPX file
    with open(map_file_path, 'r', encoding='utf-8') as f:
        gpx = gpxpy.parse(f)

    # 2) Extract points
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                points.append(p)

    StepDist = []
    StepElevation = []
    StepAngle = []
    for i in range(len(points)-1):
        lat1, lon1, ele1 = points[i].latitude,   points[i].longitude,   points[i].elevation
        lat2, lon2, ele2 = points[i+1].latitude, points[i+1].longitude, points[i+1].elevation

        # Approx distance (Haversine)
        R = 6371000
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = (math.sin(dLat/2)**2 +
             math.cos(math.radians(lat1)) *
             math.cos(math.radians(lat2)) *
             math.sin(dLon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        dist_2d = R*c  

        delta_ele = ele2 - ele1
        totaldist = math.sqrt(dist_2d**2 + delta_ele**2)

        StepDist.append(totaldist)
        StepElevation.append(ele2) 
        slope = (delta_ele / dist_2d) if dist_2d != 0 else 0
        angle = math.atan(slope)
        StepAngle.append(angle)

    StepRRcoef      = [RRcoef_input]*len(StepDist)
    ReduceSpeedDist = [0]*len(StepDist)
    V_max_LatAcc    = [Ay_max]*len(StepDist)
    V_max_XRoads    = [99]*len(StepDist)

    return (StepDist, StepElevation, StepAngle,
            StepRRcoef, ReduceSpeedDist, V_max_LatAcc, V_max_XRoads)



def SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                                     Dist, Steps, StepDist, ll, i):
    """
    If the cyclist's current speed 'vx' exceeds the segment's lateral-acc-based 
    threshold V_max_LatAcc_individ[i], then we return 1 -> reduce speed.
    Otherwise return 0. 
    """
    # If we are above the lateral-acc-based speed limit, need to decelerate.
    if vx > V_max_LatAcc_individ[i]:
        return 1
    else:
        return 0


def SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads,
                                     Dist, Steps, StepDist, ll, i,
                                     v_x_total, v_x):
    """
    Checks if the cyclist is approaching an intersection that forces 
    a certain maximum speed or even a stop. We return:
      ReduceSpeedStatus in {0,1,2,3,4}, 
      v_x_target, 
      ax, 
      StopFlag, 
      SpeedReductionTable 
    depending on what we want.

    Example logic:
      If V_max_XRoads[i] < vx => we must decelerate => set ReduceSpeedStatus=1
      or if V_max_XRoads[i] is really small => maybe stop.

    This example tries to keep it very simple.
    """
    # default
    ReduceSpeedStatus = 0
    v_x_target = vx
    ax = 0.0
    StopFlag = 0
    SpeedReductionTable = 0

    # For demonstration:
    # If V_max_XRoads[i] < 1 => means we must come to a stop
    # If V_max_XRoads[i] < vx => means we must decelerate
    if V_max_XRoads[i] is not None and V_max_XRoads[i] < 1.0:
        # forced stop
        ReduceSpeedStatus = 1
        StopFlag = 1
        ax = ax_Dec_adapt
        SpeedReductionTable = 4  # referencing e.g. a "stop sign"
    elif V_max_XRoads[i] < vx:
        # forced slowdown
        ReduceSpeedStatus = 1
        ax = ax_Dec_adapt
        # we can guess speedReductionTable=1 for a yield sign, etc.
        SpeedReductionTable = 2

    return (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable)


def PowerDeceleration(m, RRcoef, angle, vx, cwxA, rho, P, ax):
    """
    Called when cyclist is decelerating intentionally (braking or slowing).
    In the original Matlab code, negative acceleration means the cyclist's 
    input power is effectively zero, and the difference is dissipated by the brake.

    Return the partial powers:
      P_in, P_roll, P_air, P_climb, P_acc
    so that your script can track how total energy is distributed.
    """
    # Resistive forces:
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    # We define the partial powers as those forces * vx:
    P_roll  = F_roll * vx
    P_air   = F_air  * vx
    # For climb, it's negative if angle<0 => going downhill
    # We'll store as 'power for climbing':
    P_climb = F_climb*vx
    # If we are actively braking, input power is zero:
    P_in = 0.0
    # The "acc power" from the cyclist is negative if ax<0 => 
    # but we track it as 0 if we want to sum "cyclist's positive input energy."
    # We'll track the mechanical 'acc' portion:
    P_acc = m*ax*vx  # Will be negative if ax<0

    # Because deceleration => P_in = 0 from the cyclist perspective
    return (P_in, P_roll, P_air, P_climb, P_acc)


def PowerInputOff(m, RRcoef, angle, vx, cwxA, rho):
    """
    The cyclist is coasting (no pedaling). The net acceleration is negative 
    if the sum of resistive forces > 0, or positive if downhill is steep enough.

    Return ax (m/s^2).
    ax = (F_drive - F_resist) / m, but F_drive=0 because input is off. 
    """
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    # total resistive force
    F_resist = F_roll + F_air + F_climb
    # sign depends on whether the slope is downward or upward:
    # If slope is negative enough, F_climb might be negative => accelerates.
    ax = -F_resist / m
    return ax


def PowerInputOn(m, RRcoef, angle, vx, cwxA, rho, P, V_max):
    """
    Cyclist is actively pedaling with power = P. 
    1) compute rolling/air/climb power demands at speed vx
    2) leftover = P - (sum of those demands)
    3) that leftover is for acceleration, if positive. If negative => speed will 
       actually drop, but usually the code tries a different logic if leftover<0.
    Return: 
      ax, P_in, P_roll, P_air, P_climb, P_acc
    in the same format as your Matlab code.
    """
    # Resistive forces => partial powers at speed vx:
    F_roll  = m*9.81*math.cos(angle)*RRcoef
    F_air   = 0.5*cwxA*rho*(vx**2)
    F_climb = m*9.81*math.sin(angle)
    P_roll  = F_roll * vx
    P_air   = F_air  * vx
    P_climb = F_climb*vx

    # sum of resistances
    ResistPower = P_roll + P_air + P_climb
    # if P < ResistPower => can't maintain that speed => net deceleration 
    # but this function is "PowerInputOn," so let's just compute leftover:
    leftover = P - ResistPower
    if leftover < 0:
        # we don't forcibly handle negative leftover here because 
        # the main loop might do a separate call to a decel function.
        # but let's let P_acc go negative so total is correct:
        P_acc = leftover if vx>0 else 0.0
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P
    else:
        # leftover power -> acceleration
        P_acc = leftover
        # ax = (Power / (vx)) / m 
        ax = (P_acc / vx)/m if vx>0 else 0.0
        P_in = P

    return (ax, P_in, P_roll, P_air, P_climb, P_acc)


def FreeRollingSlope(m, RRcoef, cwxA, rho):
    """
    For a range of speeds (0..some max), compute the slope angle alpha 
    for which the net force is zero if the cyclist has no input power 
    and isn't braking. Then the speed would be 'free rolling' equilibrium 
    on that slope.

    Return alpha_Vector, vx_Vector so the main code can see 
    at what slope a given speed is "free rolling." 
    """
    vx_Vector = [v*0.5 for v in range(1, 41)]  # from 0.5 to 20 m/s in 0.5 steps
    alpha_Vector = []
    for vx in vx_Vector:
        # Resistive force:
        F_roll = m*9.81*RRcoef
        F_air  = 0.5*cwxA*rho*(vx**2)
        # If net is zero, sum of rolling+air = component of gravity 
        # => m*g*sin(alpha) = F_roll + F_air
        # => sin(alpha) = (F_roll + F_air)/(m*g)
        # We'll clip if >1 or < -1
        top = (F_roll + F_air)/ (m*9.81)
        if top > 1.0:
            alpha = math.pi/2.0
        elif top < -1.0:
            alpha = -math.pi/2.0
        else:
            alpha = math.asin(top)
        alpha_Vector.append(alpha)
    return alpha_Vector, vx_Vector

# ------------------------------------------------------------------------
# MAIN FUNCTION: BikeEnergyModel
# This directly translates your Matlab function to Python
# ------------------------------------------------------------------------
def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path):
    """
    Now we accept 'map_file_path' from outside. We do NOT hardcode the GPX file.
    """
    # Constants
    temp = 20.0 
    rho  = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926

    V_max         = 10.5     
    Ay_max        = 2        
    ax_Dec_adapt  = -0.3
    ax_Dec_LatAcc = -1.5

    # Import map data from the caller's path
    (StepDist, StepElevation, StepAngle, StepRRcoef,
     ReduceSpeedDist, V_max_LatAcc, V_max_XRoads) = MapData(
        map_file_path, 0, CrIn, Ay_max, temp
    )

    Steps = len(StepDist)
    if Steps != len(StepElevation):
        print(' Error in MapData function: length mismatch in StepElevation')
    elif Steps != len(StepRRcoef):
        print(' Error in MapData function: length mismatch in StepRRcoef')
    elif Steps != len(V_max_LatAcc):
        print(' Error in MapData function: length mismatch in V_max_LatAcc')

    m = CyclistMassIn + 18.3
    P = CyclistPowerIn - 5
    P_up = CyclistPowerIn*1.5 - 5
    V_max_LatAcc_individ = V_max_LatAcc

    alpha_Vector, vx_Vector = FreeRollingSlope(m, StepRRcoef[0], cwxA, rho)
    # Try to find index
    try:
        V_max_Index = vx_Vector.index(V_max)
        Alpha_Vmax_steadystate = alpha_Vector[V_max_Index]
    except ValueError:
        print(f"Warning: V_max={V_max} not found in vx_Vector, set alpha=0.0")
        Alpha_Vmax_steadystate = 0.0

    Time = 0.0
    Energy = 0.0
    Energy_roll  = 0.0
    Energy_air   = 0.0
    Energy_climb = 0.0
    Energy_acc   = 0.0

    v_x_total                = []
    vx = 5.0
    StopFlag = 0
    ax = 0.0

    for i in range(Steps):
        Dist = round(StepDist[i]/0.01)
        for ll in range(Dist):
            ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(
                vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                Dist, Steps, StepDist, ll, i
            )
            (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable) = \
                SpeedReductionCausedByCrossRoads(
                    vx, ax_Dec_adapt, V_max_XRoads, Dist, Steps, StepDist, ll, i,
                    v_x_total, vx
                )

            P_roll = 0.0
            P_air  = 0.0
            P_climb= 0.0
            P_acc  = 0.0
            P_in   = 0.0

            if (ReduceSpeedStatus == 1 or StopFlag == 1):
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedFlag == 1:
                ax = ax_Dec_LatAcc
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )
            elif ReduceSpeedStatus == 2:
                P_in = 0.0
                ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
            else:
                # handle slope vs free rolling
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    if vx > V_max:
                        ax = ax_Dec_adapt
                        P_in = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )
                else:
                    if vx > V_max:
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )

            # update speed
            new_vx_sq = vx**2 + 2.0 * ax * 0.01
            if new_vx_sq < 0:
                vx = 0.0
            else:
                vx = math.sqrt(new_vx_sq)

            if vx < 0.01 and StopFlag == 1:
                StopFlag = 0

            if vx > 0:
                dt = 0.01/vx
                Energy += P_in * dt
                Time   += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air   * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc   * dt

        v_x_total.extend([vx]*Dist)

    Distance = len(v_x_total)/100.0
    AvgSpeed = Distance/Time if Time>0 else 0.0
    return (Energy, Time, Distance, AvgSpeed)


# ---------------  Example usage if you wish to run this ---------------
#if __name__ == "__main__":
#    E, T, D, V = BikeEnergyModel(150, 70, 0.007, 0.45)
#    print("Energy:", E, "J")
#    print("Time:", T, "s")
#    print("Distance:", D, "m")
#    print("AvgSpeed:", V, "m/s =>", V*3.6, "km/h")
