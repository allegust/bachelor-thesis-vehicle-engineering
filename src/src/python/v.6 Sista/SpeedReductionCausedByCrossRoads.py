def Speed_Reduction_Caused_By_CrossRoads(
    vx_current, ax_Dec_adapt, V_max_XRoads, Dist, Steps, StepDist, ll, i,
    v_x_total, v_x_list):
    """
    Speed reduction caused by cross roads ahead

    vx_VirtualPreview mit einem festgelegten neg ax (ax_Dec_LatAcc) zum nächsten 
    (und zum übernächsten, und zum ...) Step berechnen
    wenn vx_VirtualPreview >= V_max_LatAcc(i+1) (und i+2 ...) -> festgelegte neg ax einleiten
    """

    # Constants
    g = 9.81  # [m/s^2]
    ax_Dec_Stop = -2  # [m/s^2]

    # Defaults
    v_x_target = vx_current   # [m/s] 
    ax = ax_Dec_adapt # [m/s^2]
    StopFlag = 0
    ReduceSpeedStatus = 0 # no need to reduce speed

    # assume that the LatAcc-Speedlimit ahead is close to zero
    # then there is need for a sufficient long braking distance s_max
    # Deceleration distance: s = 0.5 * v_diff^2 / a
    s_max = -0.5 * (vx_current**2) / ax_Dec_adapt  # ax_Dec_adapt is negative but s_max is positive

    # Actual distance driven:
    Dist_now = (len(v_x_total) + len(v_x_list)) / 100.0  # [m] length(...) returns #elements => #cm

    # Dist_find is True where Dist_now <= V_max_XRoads(:,1)
    Dist_find = [Dist_now <= row[0] for row in V_max_XRoads]
    # find the first index where Dist_find is True
    Dist_find_logix = None
    for idx, val in enumerate(Dist_find):
        if val:
            Dist_find_logix = idx
            break

    if Dist_find_logix is None:  
        # if the algorithm went through all the brake-points, Dist_find_logix is empty
        ReduceSpeedStatus = 0  # no need to reduce speed
        SpeedReductionTable = 0  # Indicator according MapData table for paper
        return ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable

    Dist2Slow = V_max_XRoads[Dist_find_logix, 0] - Dist_now  # distance to the point where to reach the aimed velocity

    # Define the velocity aimed for
    if (V_max_XRoads[Dist_find_logix, 1] == 1) and (Dist2Slow < 20) and (vx_current > 5):
        # free rolling at next road piece where to slow down
        # distance < 20m and current speed > 5m/s
        ReduceSpeedStatus = 2  # rolling free, P_input=0
    elif V_max_XRoads[Dist_find_logix, 1] == 2:
        # brake to 4 m/s
        v_x_target = 4
        BrakeDist = -((vx_current**2 - v_x_target**2) / (2 * ax_Dec_Stop))
        if (vx_current > v_x_target) and (BrakeDist >= Dist2Slow):
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
    elif V_max_XRoads[Dist_find_logix, 1] == 3:
        # brake to 2 m/s
        v_x_target = 2
        BrakeDist = -((vx_current - v_x_target)**2 / (2 * ax_Dec_Stop))
        if (vx_current > v_x_target) and (BrakeDist >= Dist2Slow):
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
    elif V_max_XRoads[Dist_find_logix, 1] == 4:
        # stop sign / traffic light => brake to 0 m/s
        v_x_target = 0
        BrakeDist = -((vx_current - v_x_target)**2 / (2 * ax_Dec_Stop))
        if (vx_current > v_x_target) and (BrakeDist >= Dist2Slow):
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
            StopFlag = 1
    else:
        ReduceSpeedStatus = 0

    SpeedReductionTable = V_max_XRoads[Dist_find_logix, 1]  # Indicator according MapData table for paper
    return ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable
