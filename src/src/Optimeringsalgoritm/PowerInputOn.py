import math

import math

def Power_Input_On(m, StepRRcoef, StepAngle, vx, cwxA, rho, P, V_max):
    """
    This function calculates the longitudinal acceleration if the model is slower than desired,
    matching the MATLAB version exactly.
    """

    # Constants
    g = 9.81  # [m/s^2]


    # Power adaptation to uphill cycling
    # factor 1 at 0%
    # factor 1.5 at 10% (5.7 degree = 0.0997 rad) (flat out)
    """
    P_origin = P
    P_new = P * 5 * StepAngle
    if P_new > 1.5 * P_origin:
        P_new = 1.5 * P_origin
    elif P_new < P_origin:
        P_new = 1.5 * P_origin
    P = P_new
    """
    P = 1.5 * P
    
    # Steady state power used
    P_roll_steady = m * g * StepRRcoef * math.cos(StepAngle) * vx
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)
    P_climb_steady = m * g * math.sin(StepAngle) * vx

    if P_climb_steady >= P:
        P_climb_steady = P
        vx = P / (m * g * math.sin(StepAngle))

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




"""""
def Power_Input_On(m, StepRRcoef, StepAngle, vx, cwxA, rho, P, V_max):

    #This function calculates the longitudinal acceleration if the model is slower than desired


    # Constants
    g = 9.81  # [m/s^2]

    # Power adaptation to uphill cycling
    # factor 1 at 0%
    # factor 1.5 at 10% (5.7 degree = 0.0997 rad) (flat out)
    P_origin = P
    P_new = P * 5 * StepAngle
    if P_new > 1.5 * P_origin:
        P_new = 1.5 * P_origin
    elif P_new < P_origin:
        P_new = 1.5 * P_origin
    P = P_new

    # Steady state power used
    # Rolling resistance power
    P_roll_steady = m * g * StepRRcoef * math.cos(StepAngle) * vx
    # Air resistance power
    P_air_steady = 0.5 * cwxA * rho * (vx ** 3)
    # Climbing power
    P_climb_steady = m * g * math.sin(StepAngle) * vx
    if P_climb_steady >= P:
        P_climb_steady = P
        vx = P / (m * g * math.sin(StepAngle))

    SteadyStatePowerUsed = P_roll_steady + P_air_steady + P_climb_steady

    # free power left for acceleration as function of individual max speed
    if vx > 0.8 * V_max and StepAngle < 0:
        # half of steady-state power will be used
        # this is only true during downhill slope
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

"""