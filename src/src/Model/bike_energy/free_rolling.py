import math
from typing import List, Tuple


def _alpha_vx(
    m: float, f_r: float, vx: float, cwxA: float, rho: float, g: float
) -> float:
    """
    Optimize alpha for a given vx by brute-force search around downhill slopes.
    """
    alphas = [-0.001 - 0.1 * i for i in range(10)]
    F = [
        abs(m * g * f_r * math.cos(a) * vx
            + 0.5 * cwxA * rho * vx**3
            + m * g * math.sin(a) * vx)
        for a in alphas
    ]
    idx = F.index(min(F))

    if idx == 0 or (idx < 9 and F[idx+1] < F[idx-1]):
        a_min, a_max = alphas[idx], alphas[idx+1]
    else:
        a_min, a_max = alphas[idx-1], alphas[idx]

    alphas2 = [a_min + i * (a_max - a_min)/10 for i in range(11)]
    F2 = [
        abs(m * g * f_r * math.cos(a) * vx
            + 0.5 * cwxA * rho * vx**3
            + m * g * math.sin(a) * vx)
        for a in alphas2
    ]
    return alphas2[F2.index(min(F2))]


def free_rolling_slope(
    m: float, f_r: float, cwxA: float, rho: float, g: float
) -> Tuple[List[float], List[float]]:
    """
    For vx in [0.1,0.2,...,20], find the downhill slope alpha at which
    rolling+air drag = downhill power. Returns (alpha_list, vx_list).
    """
    vx_list = [0.1 * i for i in range(1, 201)]
    alpha_list = [_alpha_vx(m, f_r, vx, cwxA, rho, g) for vx in vx_list]
    return alpha_list, vx_list
