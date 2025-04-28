import math
def Power_Input_Off(m, StepRRcoef, StepAngle, vx, cwxA, rho):
    """
    This function calculates the longitudinal negative acceleration if the model is faster than desired
    """

    # Constants
    g = 9.81  # [m/s^2]
    P = 0     # power off / free rolling

    # Rolling resistance power
    P_roll_steady = m * g * StepRRcoef * math.cos(StepAngle) * vx
    # Air resistance power
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)
    # Climbing power
    P_climb_steady = m * g * math.sin(StepAngle) * vx
    if P_climb_steady >= P:
        P_climb_steady = P

    SteadyStatePowerUsed = P_roll_steady + P_air_steady + P_climb_steady

    # free power left for acceleration
    P_acc = P - SteadyStatePowerUsed

    # calculate acceleration based on free power
    ax = P_acc / (m * vx)

    return ax
