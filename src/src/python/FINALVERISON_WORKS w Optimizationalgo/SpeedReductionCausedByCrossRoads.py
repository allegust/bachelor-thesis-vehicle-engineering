# Updated SpeedReductionCausedByCrossRoads.py

def Speed_Reduction_Caused_By_CrossRoads(
    vx_current, ax_Dec_adapt, V_max_XRoads, Dist, Steps,
    StepDist, ll, i, v_x_total, v_x_list
):
    """
    Speed reduction caused by cross roads ahead.
    Fixed to compute the actual distance travelled in metres.
    """
    # Constants
    g = 9.81  # [m/s^2]
    ax_Dec_Stop = -2.0  # [m/s^2]

    # Defaults
    v_x_target = vx_current
    ax = ax_Dec_adapt
    StopFlag = 0
    ReduceSpeedStatus = 0

    # Compute total centimetre slices done so far
    cm_done = sum(len(seg) for seg in v_x_total) + len(v_x_list)
    Dist_now = cm_done * 1 #/ 100.0  # convert to metres

    # Find the upcoming crossroad entry in the table
    next_idx = next((idx for idx, row in enumerate(V_max_XRoads) if Dist_now <= row[0]), None)
    if next_idx is None:
        # No more stops ahead
        return ReduceSpeedStatus, v_x_target, ax, StopFlag, 0

    Dist2Slow = V_max_XRoads[next_idx][0] - Dist_now
    status = V_max_XRoads[next_idx][1]

    if status == 1 and Dist2Slow < 20 and vx_current > 5:
        # free rolling
        ReduceSpeedStatus = 2
    elif status == 2:
        # brake to 4 m/s
        v_x_target = 4
        BrakeDist = -((vx_current**2 - v_x_target**2) / (2.0 * ax_Dec_Stop))
        if vx_current > v_x_target and BrakeDist >= Dist2Slow:
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
    elif status == 3:
        # brake to 2 m/s
        v_x_target = 2
        BrakeDist = -((vx_current - v_x_target)**2 / (2.0 * ax_Dec_Stop))
        if vx_current > v_x_target and BrakeDist >= Dist2Slow:
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
    elif status == 4:
        # stop sign / traffic light -> brake to 0
        v_x_target = 0
        BrakeDist = -((vx_current - v_x_target)**2 / (2.0 * ax_Dec_Stop))
        if vx_current > v_x_target and BrakeDist >= Dist2Slow:
            ReduceSpeedStatus = 1
            ax = ax_Dec_Stop
            StopFlag = 1

    SpeedReductionTable = status
    return ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable
