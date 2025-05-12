import math
import numpy as np
from scipy.stats import norm
from itertools import product
from typing import Dict, Any

import time  # Add this import for timing of simulate_energy

from bike_energy.config import (
    WEIGHT_PERCENTILES,
    WEIGHT_MEAN_WOMEN,
    WEIGHT_Q25_WOMEN,
    WEIGHT_Q75_WOMEN,
    WEIGHT_MEAN_MEN,
    WEIGHT_Q25_MEN,
    WEIGHT_Q75_MEN,
    VO2_DECILES_WOMEN,
    VO2_DECILES_MEN,
    POWER_FACTOR,
    DRIVETRAIN_LOSS,
    RR_WEIBULL_LAMBDA,
    RR_WEIBULL_K,
    CWXA_VALUES,
    V_MAX,
    AX_ADAPT,
    AX_LATACC,
    AY_MAX,
    AIR_DENSITY_A,
    AIR_DENSITY_B,
    AIR_DENSITY_C,
    GPX_FOLDER,
    GPX_FILE
)
from bike_energy.map_data import map_data
from bike_energy.free_rolling import free_rolling_slope
from bike_energy.simulator import simulate_energy
from pathlib import Path

def _normal_distribution_deciles(mean_vals, q25_vals, q75_vals, percentiles):
    mu = float(np.mean(mean_vals))
    q25 = float(np.mean(q25_vals))
    q75 = float(np.mean(q75_vals))
    z25 = norm.ppf(0.25)
    z75 = norm.ppf(0.75)
    sigma = (q75 - q25) / (z75 - z25)
    return [float(norm.ppf(p, loc=mu, scale=sigma)) for p in percentiles]

def _weibull_inverse(percentiles, lambda_, k):
    return [lambda_ * (-math.log(1 - p)) ** (1 / k) for p in percentiles]

def EnergyController() -> Dict[str, Any]:
    deciles_w = _normal_distribution_deciles(
        WEIGHT_MEAN_WOMEN, WEIGHT_Q25_WOMEN, WEIGHT_Q75_WOMEN, WEIGHT_PERCENTILES
    )
    deciles_m = _normal_distribution_deciles(
        WEIGHT_MEAN_MEN, WEIGHT_Q25_MEN, WEIGHT_Q75_MEN, WEIGHT_PERCENTILES
    )

    CRin = _weibull_inverse(WEIGHT_PERCENTILES, RR_WEIBULL_LAMBDA, RR_WEIBULL_K)
    cwxA_In = CWXA_VALUES

    gpx_path = Path(GPX_FOLDER) / GPX_FILE

    results: Dict[str, Any] = {}
    """for gender, deciles, vo2_list in [
        ("women", deciles_w, VO2_DECILES_WOMEN),
        ("men", deciles_m, VO2_DECILES_MEN),
    ]:
        pspec = [v * POWER_FACTOR / 1000.0 for v in vo2_list]
        P_flat = [0.3 * p for p in pspec]

        E_list, T_list, D_list, V_list = [], [], [], []
        for P, M, cr, cwx in product(P_flat, deciles, CRin, cwxA_In):
            rho = AIR_DENSITY_A * 20.0 ** 2 + AIR_DENSITY_B * 20.0 + AIR_DENSITY_C
            (
                step_dist,
                step_ele,
                slope_angle,
                step_rr,
                _reduce_speed_dist,
                v_max_latacc,
                v_max_xroads,
            ) = map_data(str(gpx_path), cr, AY_MAX, 20.0)

            if step_dist.size == 0:
                continue

            m_total = M + 18.3
            alpha_vec, vx_vec = free_rolling_slope(m_total, step_rr[0], cwx, rho)
            idx = int(np.argmin(np.abs(vx_vec - V_MAX)))
            alpha_vmax_ss = alpha_vec[idx]"""
    for gender, deciles, vo2_list in [
        ("women", deciles_w, VO2_DECILES_WOMEN),
        ("men",   deciles_m, VO2_DECILES_MEN),
    ]:
        # specific power [W/kg]
        pspec = [v * POWER_FACTOR / 1000.0 for v in vo2_list]

        E_list, T_list, D_list, V_list = [], [], [], []
        # now include spec_power and mass in the same loop
        for spec_power, M, cr, cwx in product(pspec, deciles, CRin, cwxA_In):
            # compute absolute max power [W]
            absolute_max_power = spec_power * M
            # operate at 30% of that
            P_flat = 0.3 * absolute_max_power

            # compute air density at 20 m/s
            rho = AIR_DENSITY_A * 20.0 ** 2 + AIR_DENSITY_B * 20.0 + AIR_DENSITY_C
            (
                step_dist,
                step_ele,
                slope_angle,
                step_rr,
                _reduce_speed_dist,
                v_max_latacc,
                v_max_xroads,
            ) = map_data(str(gpx_path), cr, AY_MAX, 20.0)

            if step_dist.size == 0:
                continue

            m_total = M + 18.3
            alpha_vec, vx_vec = free_rolling_slope(m_total, step_rr[0], cwx, rho)
            idx = int(np.argmin(np.abs(vx_vec - V_MAX)))
            alpha_vmax_ss = alpha_vec[idx]




            p_flat = P_flat - DRIVETRAIN_LOSS

            #start_time = time.perf_counter()  # Start timing   
            E, T, D, Vavg, *_ = simulate_energy(
                step_dist,
                slope_angle,
                step_rr,
                v_max_latacc,
                v_max_xroads,
                m_total,
                p_flat,
                V_MAX,
                AX_ADAPT,
                AX_LATACC,
                cwx,
                rho,
                alpha_vmax_ss,
            )
            #elapsed_time = time.perf_counter() - start_time  # End timing
            #print(f"simulate_energy execution time: {elapsed_time:.6f} seconds")  # Log the time
            E_list.append(E)
            T_list.append(T)
            D_list.append(D)
            V_list.append(Vavg)

        results[f"energy_{gender}"] = sorted(E_list)
        results[f"time_{gender}"] = sorted(T_list)
        results[f"distance_{gender}"] = sorted(D_list)
        results[f"avg_speed_{gender}"] = sorted(V_list)

    return results
