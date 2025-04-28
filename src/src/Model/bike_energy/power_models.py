import math
from typing import Tuple


def power_input_on(
    m: float,
    rr_coef: float,
    step_angle: float,
    vx: float,
    cwxA: float,
    rho: float,
    p_flat: float,
    v_max: float,
    g: float,
) -> Tuple[float, float, float, float, float, float]:
    """
    When cyclist is powering: return
    (ax, P_in, P_roll, P_air, P_climb, P_acc).
    """
    P_origin = p_flat
    P_new = p_flat * 5.0 * step_angle
    if P_new > 1.5 * P_origin:
        P = 1.5 * P_origin
    elif P_new < P_origin:
        P = 1.5 * P_origin
    else:
        P = P_new

    P_roll  = m * g * rr_coef * math.cos(step_angle) * vx
    P_air   = 0.5 * cwxA * rho * vx**3
    P_climb = m * g * math.sin(step_angle) * vx
    if P_climb >= P:
        P_climb = P
        vx = P / (m * g * math.sin(step_angle))

    used = P_roll + P_air + P_climb

    if vx > 0.8 * v_max and step_angle < 0:
        P_acc = P/2 - used
    else:
        P_acc = P - used

    ax = P_acc / (m * vx) if vx > 0 else P_acc / (m * 0.01)
    ax = min(ax, 0.1) if step_angle >= 0.05 else min(ax, 0.3)

    P_acc_in = m * ax * vx if vx > 0 else m * ax * 0.01
    P_in = P_roll + P_air + P_climb + P_acc_in
    if P_in < 0:
        P_in = 0.0

    return ax, P_in, P_roll, P_air, P_climb, P_acc_in


def power_input_off(
    m: float, rr_coef: float, step_angle: float, vx: float, cwxA: float, rho: float, g: float
) -> float:
    """
    Free-rolling: returns deceleration ax s.t. rolling+air+climb = 0.
    """
    P_roll  = m * g * rr_coef * math.cos(step_angle) * vx
    P_air   = 0.5 * cwxA * rho * vx**3
    P_climb = m * g * math.sin(step_angle) * vx
    if P_climb >= 0:
        P_climb = 0.0

    used = P_roll + P_air + P_climb
    P_acc = -used
    ax = P_acc / (m * vx)
    return ax


def power_deceleration(
    m: float,
    rr_coef: float,
    step_angle: float,
    vx: float,
    cwxA: float,
    rho: float,
    p_flat: float,
    ax_brake: float,
    g: float,
) -> Tuple[float, float, float, float, float]:
    """
    When braking: return (P_in, P_roll, P_air, P_climb, P_acc).
    """
    P_climb = m * g * math.sin(step_angle) * vx
    if P_climb >= p_flat:
        P_climb = p_flat

    P_roll = m * g * rr_coef * math.cos(step_angle) * vx
    P_air  = 0.5 * cwxA * rho * vx**3
    used   = P_roll + P_air + P_climb

    P_acc = ax_brake * m * vx
    P_in  = used + P_acc
    if P_in < 0:
        P_in = 0.0

    return P_in, P_roll, P_air, P_climb, P_acc
