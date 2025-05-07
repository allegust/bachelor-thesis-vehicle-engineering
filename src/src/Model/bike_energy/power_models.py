import math
from .config import gravity as G, ax_stop, ax_adapt, ax_latacc



def power_input_on(m, StepRRcoef, StepAngle, vx, cwxA, rho, P, V_max):
    """
    This function calculates the longitudinal acceleration if the model is slower than desired,
    matching the MATLAB version exactly.
    """

    # Constants
    #G = 9.81  # [m/s^2]


    # Power adaptation to uphill cycling
    # factor 1 at 0%
    # factor 1.5 at 10% (5.7 degree = 0.0997 rad) (flat out)


    P_origin = P                   # rider’s constant flat-road power
    P_new    = P * 5.0 * StepAngle # uphill adaptation

    if P_new > 1.5 * P_origin:
        P = 1.5 * P_origin
    elif P_new < P_origin:
        P = 1.5 * P_origin    # bump up any “too small” P_new to 150% flat power
    else:
        P = P_new             # only in the middle range do you actually use P_new
    
    # Steady state power used
    P_roll_steady = m * G() * StepRRcoef * math.cos(StepAngle) * vx
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)
    P_climb_steady = m * G() * math.sin(StepAngle) * vx

    if P_climb_steady >= P:
        P_climb_steady = P
        vx = P / (m * G() * math.sin(StepAngle))

    SteadyStatePowerUsed = P_roll_steady + P_air_steady + P_climb_steady

    # free power left for acceleration as function of individual max speed
    if vx > 0.8 * V_max and StepAngle < 0:
        P_acc = (P / 2.0) - SteadyStatePowerUsed
    else:
        P_acc = P - SteadyStatePowerUsed

    # calculate acceleration based on free power
    if vx != 0:
        ax = P_acc / (m * vx)
    else:
        ax = P_acc / (m * 0.01)

    # limit acceleration to a meaningful amount
    if StepAngle >= 0.05 and ax > 0.1:
        ax = 0.1
    elif StepAngle < 0.05 and ax > 0.3:
        ax = 0.3

    # calculate the effective acceleration power
    if vx != 0:
        P_acc_in = m * ax * vx
    else:
        P_acc_in = m * ax * 0.01

    # calculate the effective cyclist input power
    P_in = P_roll_steady + P_air_steady + P_climb_steady + P_acc_in

    if P_in <= 0:
        P_in = 0

    return ax, P_in, P_roll_steady, P_air_steady, P_climb_steady, P_acc_in


def power_input_off(m, StepRRcoef, StepAngle, vx, cwxA, rho):
    """
    This function calculates the longitudinal negative acceleration if the model is faster than desired
    """

    # Constants
    #G = 9.81  # [m/s^2]
    P = 0     # power off / free rolling

    # Rolling resistance power
    P_roll_steady = m * G() * StepRRcoef * math.cos(StepAngle) * vx
    # Air resistance power
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)
    # Climbing power
    P_climb_steady = m * G() * math.sin(StepAngle) * vx
    if P_climb_steady >= P:
        P_climb_steady = P

    SteadyStatePowerUsed = P_roll_steady + P_air_steady + P_climb_steady

    # free power left for acceleration
    P_acc = P - SteadyStatePowerUsed

    # calculate acceleration based on free power
    ax = P_acc / (m * vx)

    return ax


def power_deceleration(m, StepRRcoef, StepAngle, vx, cwxA, rho, P, ax):
    """
    This function calculates the necessary power in case of deceleration
    """
    
    # Constants
    #G = 9.81  # [m/s^2]

    # Climbing power preliminary
    P_climb_steady = m * G() * math.sin(StepAngle) * vx
    if P_climb_steady >= P:
        P_climb_steady = P
        vx = P / (m * G() * math.sin(StepAngle))

    # Rolling resistance power
    P_roll_steady = m * G() * StepRRcoef * math.cos(StepAngle) * vx
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
