from bike_energy.config import AX_STOP , STEP_SIZE, AX_ADAPT, AX_LATACC

def speed_reduction_caused_by_crossRoads(
    vx_current, ax_Dec_adapt, V_max_XRoads, Dist, Steps,
    StepDist, ll, i, v_x_total, v_x_list
):
    """
    Speed reduction caused by cross roads ahead.
    Fixed to compute the actual distance travelled in metres.
    """

    ax_Dec_Stop = AX_STOP  # [m/s^2]

    # Defaults
    v_x_target = vx_current
    ax = ax_Dec_adapt
    StopFlag = 0
    ReduceSpeedStatus = 0

    # Compute total centimetre slices done so far
    cm_done = sum(len(seg) for seg in v_x_total) + len(v_x_list)
    Dist_now = cm_done * STEP_SIZE          #  / 100.0  # convert to metres

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

def speed_reduction_caused_by_high_lat_acc(vx, ax_Dec_LatAcc, V_max_LatAcc, Dist, Steps, StepDist, ll, i):

    #Speed reduction caused by high lateral acceleration during cornering ahead.

    # Maximum braking look‑ahead distance (positive value)
    s_max = -0.5 * (vx ** 2) / ax_Dec_LatAcc

    # Distance to end of current 1cm step
    Dist2EndOfStep = [(Dist - (ll + 1)) * STEP_SIZE] # / 100.0

    # Initial brake distance using the current step's lateral‑limit
    BrakeDist = [ -((vx - V_max_LatAcc[i]) ** 2) / (2.0 * ax_Dec_LatAcc) ]

    WhileCounter = 0
    # Extend look‑ahead as long as we need more braking distance
    while max(Dist2EndOfStep) < s_max:
        if (i + WhileCounter) < Steps:
            # add the next step's distance in meters
            Dist2EndOfStep.append(
                Dist2EndOfStep[WhileCounter] + StepDist[i + WhileCounter]
            )
            # compute brake dist for that step's limit
            BrakeDist.append(
                -((vx - V_max_LatAcc[i + WhileCounter]) ** 2)
                / (2.0 * ax_Dec_LatAcc)
            )
            WhileCounter += 1
        else:
            break

    # Find if any look‑ahead triggers braking
    BrakeDistLogix = [d <= b for d, b in zip(Dist2EndOfStep, BrakeDist)]
    BrakeDistLogixIndex = next((idx for idx, flag in enumerate(BrakeDistLogix) if flag), None)

    # Determine the minimum lateral-limit over the look‑ahead window
    end_idx = i + WhileCounter + 1
    if end_idx <= len(V_max_LatAcc):
        ahead_limits = V_max_LatAcc[i:end_idx]
    else:
        ahead_limits = [V_max_LatAcc[i]]

    # Trigger a speed reduction if we're above the min look‑ahead limit and no brake‑distance hit
    if (vx > min(ahead_limits)) and (BrakeDistLogixIndex is None):
        ReduceSpeedFlag = 1
    else:
        ReduceSpeedFlag = 0

    return ReduceSpeedFlag
