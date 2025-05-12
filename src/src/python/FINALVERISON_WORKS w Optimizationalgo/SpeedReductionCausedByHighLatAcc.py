"""def Speed_Reduction_Caused_By_High_Lat_Acc(vx, ax_Dec_LatAcc, V_max_LatAcc, Dist, Steps, StepDist, ll, i):
    
    #Determine if we must brake for a cornering lateral-acceleration limit.
    #Only considers the remaining distance in this 1 cm slice.
    
    # braking look-ahead distance (m) – positive
    s_max = -0.5 * vx * vx / ax_Dec_LatAcc

    # remaining distance in this segment (m): (Dist = total 1 cm slices, ll = current slice idx)
    STEP_SIZE = 0.01
    remaining = (Dist - ll) * STEP_SIZE

    # if we still have more room than we need to brake, no corner braking
    if remaining >= s_max:
        return 0

    # otherwise, if our current speed exceeds this segment’s corner limit, we must brake
    return 1 if vx > V_max_LatAcc[i] else 0"""


def Speed_Reduction_Caused_By_High_Lat_Acc(vx, ax_Dec_LatAcc, V_max_LatAcc, Dist, Steps, StepDist, ll, i):

    #Speed reduction caused by high lateral acceleration during cornering ahead.
    #Fixed to use zero-based indexing for V_max_LatAcc and StepDist.

    # Constants
    g = 9.81  # [m/s^2]

    # Maximum braking look‑ahead distance (positive value)
    s_max = -0.5 * (vx ** 2) / ax_Dec_LatAcc

    # Distance to end of current 1cm step
    Dist2EndOfStep = [(Dist - (ll + 1)) * 1] # [(Dist - (ll + 1)) / 100.0]

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
