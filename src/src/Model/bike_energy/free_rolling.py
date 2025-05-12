import math
import numpy as np
from bike_energy.config import GRAVITY as g #, ax_stop, ax_adapt, ax_latacc

def _alpha_vx(m: float, f_r: float, vx: float, cwxA: float, rho: float) -> float:
    """
    Find the slope angle α (negative = downhill) at which rolling+air drag
    exactly balances downhill power for a given speed vx.
    Uses a three‐stage bracketing + refinement approach.
    """

    # Stage 1: coarse grid of 10 α values from -0.001 to -0.901
    alpha_list = [-0.001 - 0.1 * i for i in range(10)]
    def total_power(alpha: float) -> float:
        return (
            m * g * f_r * math.cos(alpha) * vx
            + 0.5 * cwxA * rho * vx**3
            + m * g * math.sin(alpha) * vx
        )

    # Evaluate and find index of minimal |power|
    F  = [total_power(a) for a in alpha_list]
    idx = min(range(len(F)), key=lambda i: abs(F[i]))

    # Determine bracket for refinement
    if idx == 0 or (idx < len(F)-1 and abs(F[idx+1]) < abs(F[idx-1])):
        a_min, a_max = alpha_list[idx], alpha_list[idx+1]
    else:
        a_min, a_max = alpha_list[idx-1], alpha_list[idx]

    # Three‐stage refinement: subdivide bracket into 10 steps each time
    for _ in range(3):
        alphas = [a_min + i*(a_max-a_min)/10.0 for i in range(11)]
        Fs     = [total_power(a) for a in alphas]
        idx2   = min(range(len(Fs)), key=lambda i: abs(Fs[i]))
        # update bracket
        if idx2 == 0:
            a_min, a_max = alphas[0], alphas[1]
        elif idx2 == len(Fs)-1:
            a_min, a_max = alphas[-2], alphas[-1]
        else:
            a_min, a_max = alphas[idx2-1], alphas[idx2+1]

    # Final best estimate
    alphas = [a_min + i*(a_max-a_min)/10.0 for i in range(11)]
    Fs     = [total_power(a) for a in alphas]
    best_i = min(range(len(Fs)), key=lambda i: abs(Fs[i]))
    return alphas[best_i]


def free_rolling_slope(
    m: float,
    f_r: float,
    cwxA: float,
    rho: float
) -> tuple[list[float], np.ndarray]:
    """
    Compute the “free‐rolling” characteristic curve:
      for vx spanning 0.1→20.0 m/s in 0.1 m/s steps,
      find the slope α at which net power = 0.
    Returns (alpha_list, vx_array).
    """
    vx_array = np.arange(0.1, 20.0 + 1e-9, 0.1)
    alpha_list: list[float] = []

    for vx in vx_array:
        alpha = _alpha_vx(m, f_r, vx, cwxA, rho)
        alpha_list.append(alpha)

    return alpha_list, vx_array
