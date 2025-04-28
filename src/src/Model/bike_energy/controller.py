import math
import numpy as np
from scipy.stats import norm
from itertools import product
from typing import Dict, Any

from bike_energy.config import Params
from bike_energy.simulator import simulate_energy


def _normal_distribution_deciles(
    mean_vals, q25_vals, q75_vals, percentiles
):
    μ = float(np.mean(mean_vals))
    q25 = float(np.mean(q25_vals))
    q75 = float(np.mean(q75_vals))
    z25 = norm.ppf(0.25)
    z75 = norm.ppf(0.75)
    σ = (q75 - q25) / (z75 - z25)
    return [float(norm.ppf(p, loc=μ, scale=σ)) for p in percentiles]


def _weibull_inverse(percentiles, λ, k):
    return [λ * (-math.log(1-p))**(1/k) for p in percentiles]


def EnergyController(params: Params) -> Dict[str, Any]:
    """
    Loop over percentiles & genders, run simulate_energy,
    sort results, and return a JSON‐serializable dict.
    """
    wm = params.weight_model
    rr = params.rolling_resistance
    ad = params.aerodynamic_drag

    # 1) compute mass deciles for women & men
    deciles_w = _normal_distribution_deciles(
        wm.women.mean_values,
        wm.women.quantile25_values,
        wm.women.quantile75_values,
        wm.percentiles,
    )
    deciles_m = _normal_distribution_deciles(
        wm.men.mean_values,
        wm.men.quantile25_values,
        wm.men.quantile75_values,
        wm.percentiles,
    )

    # 2) compute rolling resistance inputs
    CRin = _weibull_inverse(
        rr.percentiles,
        rr.weibull.lambda_,
        rr.weibull.k,
    )

    cwxA_In = ad.cwxA_values
    pf = params.power_model.power_factor
    dl = params.power_model.drivetrain_loss

    results: Dict[str, Any] = {}
    for gender, deciles in (("women", deciles_w), ("men", deciles_m)):
        # VO2max → W/kg lists
        vo2 = (params.power_model.vo2max.women if gender=="women"
               else params.power_model.vo2max.men)
        pspec = [v * pf / 1000.0 for v in vo2]
        P_flat = [0.3 * p for p in pspec]

        E_list, T_list, D_list, V_list = [], [], [], []
        for P, M, cr, cwx in product(P_flat, deciles, CRin, cwxA_In):
            E, T, D, V = simulate_energy(params, M, P, cr, cwx)
            E_list.append(E); T_list.append(T)
            D_list.append(D); V_list.append(V)

        # sort and store
        results[f"energy_{gender}"]    = sorted(E_list)
        results[f"time_{gender}"]      = sorted(T_list)
        results[f"distance_{gender}"]  = sorted(D_list)
        results[f"avg_speed_{gender}"] = sorted(V_list)

    return results
