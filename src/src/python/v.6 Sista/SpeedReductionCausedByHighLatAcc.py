def Speed_Reduction_Caused_By_High_Lat_Acc(vx, ax_Dec_LatAcc, V_max_LatAcc, Dist, Steps, StepDist, ll, i):
    """
    Speed reduction caused by high lateral acceleration during cornering ahead

    vx betrachten
    vx_VirtualPreview mit einem festgelegten neg ax (ax_Dec_LatAcc) 
    zum nächsten (und zum übernächsten, und zum ...) Step berechnen
    wenn vx_VirtualPreview >= V_max_LatAcc(i+1) (und i+2 ...) -> festgelegte neg ax einleiten
    """

    # Constants
    g = 9.81  # [m/s^2]  (not actually used in this function, but included to match original code)

    # assume that the LatAcc-Speedlimit ahead is close to zero
    # then there is need for a sufficient long braking distance
    # Deceleration distance: s = 0.5 * v_diff^2 / a
    # ax_Dec_LatAcc is negative but s_max must be positive!
    s_max = -0.5 * (vx ** 2) / ax_Dec_LatAcc

    # Distance to beginning of next step / end of actual step
    # Dist and ll are in [cm], so convert to meters
    Dist2EndOfStep = [(Dist - ll) / 100.0]

    # Brake distance needed at actual speed
    # Note: i is assumed 1-based as in MATLAB, so we adjust by (i-1) for Python indexing
    BrakeDist = [
        -((vx - V_max_LatAcc[i - 1]) ** 2) / (2.0 * ax_Dec_LatAcc)
    ]

    # Distance to beginning of overnext steps
    WhileCounter = 0
    while max(Dist2EndOfStep) < s_max:
        # as long as s_max is larger than the look-ahead-distance -> increase the look-ahead-distance
        if (i + WhileCounter) < Steps:
            # extend the Dist2EndOfStep list
            Dist2EndOfStep.append(
                Dist2EndOfStep[WhileCounter] + StepDist[(i + WhileCounter) - 1]
            )
            # Brake distance needed at actual speed to respective Step
            BrakeDist.append(
                -((vx - V_max_LatAcc[(i + WhileCounter) - 1]) ** 2)
                / (2.0 * ax_Dec_LatAcc)
            )
            WhileCounter += 1
        else:
            # leave the while loop
            break

    # Compare Dist2EndOfStep to BrakeDist element-wise
    # returns a boolean list where True indicates Dist2EndOfStep <= BrakeDist
    BrakeDistLogix = [
        (dist_val <= brake_val)
        for dist_val, brake_val in zip(Dist2EndOfStep, BrakeDist)
    ]

    # Find the position of the first True that indicates the need to decelerate
    BrakeDistLogixIndex = None
    for idx, val in enumerate(BrakeDistLogix):
        if val:
            BrakeDistLogixIndex = idx
            break

    # Deceleration distance: s = 0.5 * v_diff^2 / a
    # i+WhileCounter <= length(V_max_LatAcc) in MATLAB => i+WhileCounter <= len(V_max_LatAcc) in Python
    if (i + WhileCounter) <= len(V_max_LatAcc):
        # if vx is larger than any V_max ahead AND BrakeDistLogix has found a need to decelerate
        if (vx > min(V_max_LatAcc[i : i + WhileCounter + 1])) and (BrakeDistLogixIndex is None):
            ReduceSpeedFlag = 1
        else:
            ReduceSpeedFlag = 0
    else:
        # this step is important for last entry
        # if vx is larger than any V_max ahead AND BrakeDistLogix has found a need to decelerate
        if (vx > min([V_max_LatAcc[i - 1]])) and (BrakeDistLogixIndex is None):
            ReduceSpeedFlag = 1
        else:
            ReduceSpeedFlag = 0

    return ReduceSpeedFlag
