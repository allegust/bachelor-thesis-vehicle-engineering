def Power_Deceleration(m, StepRRcoef, StepAngle, vx, cwxA, rho, P, ax):
    """
    This function calculates the necessary power in case of deceleration
    """

    import math

    # Constants
    g = 9.81  # [m/s^2]

    # Climbing power preliminary
    P_climb_steady = m * g * math.sin(StepAngle) * vx
    if P_climb_steady >= P:
        P_climb_steady = P
        vx = P / (m * g * math.sin(StepAngle))

    # Rolling resistance power
    P_roll_steady = m * g * StepRRcoef * math.cos(StepAngle) * vx
    # Air resistance power
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)

    SteadyStatePowerUsed = P_roll_steady + P_air_steady + P_climb_steady

    # Calculate acceleration power (ax is negative -> power is negative!)
    P_acc = ax * m * vx

    # Subtract the deceleration from steady-state
    P_in = SteadyStatePowerUsed + P_acc

    # Limit to zero (no negative values!)
    if P_in <= 0:
        P_in = 0

    return P_in, P_roll_steady, P_air_steady, P_climb_steady, P_acc
