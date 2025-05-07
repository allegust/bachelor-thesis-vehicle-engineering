import math
from pathlib import Path
from typing import Tuple, Union

import numpy as np

from .config import (
    STEP_SIZE,
    V_MAX,
    AY_MAX,
    AX_ADAPT,
    AX_LATACC,
    DRIVETRAIN_LOSS,
    AIR_DENSITY_A,
    AIR_DENSITY_B,
    AIR_DENSITY_C,
)
from .map_data import map_data
from .free_rolling import free_rolling_slope
from .power_models import (
    power_input_on,
    power_input_off,
    power_deceleration,
)
from .speed_control import (
    speed_reduction_caused_by_crossroads,
    speed_reduction_caused_by_high_lat_acc,
)

# Precompute inverse for performance
INV_STEP = 1.0 / STEP_SIZE


def simulate_energy(
    step_dist: np.ndarray,
    slope_angle: np.ndarray,
    step_rr: np.ndarray,
    v_max_latacc: np.ndarray,
    v_max_xroads: np.ndarray,
    mass: float,
    p_flat: float,
    v_max: float,
    ax_dec_adapt: float,
    ax_dec_latacc: float,
    cwxA: float,
    rho: float,
    alpha_vmax_ss: float,
) -> Tuple[
    float,  # total energy [J]
    float,  # total time [s]
    float,  # distance [m]
    float,  # avg speed [m/s]
    float,  # energy_roll [J]
    float,  # energy_air [J]
    float,  # energy_climb [J]
    float,  # energy_acc [J]
]:
    """
    Given all step arrays and cyclist params, run the centimetreloop simulation.
    """
    time_s = energy = energy_roll = energy_air = energy_climb = energy_acc = 0.0
    vx = 5.0  # initial speed [m/s]
    v_x_total = []

    steps = step_dist.size

    for i in range(steps):
        # number of 1cm slices in this segment
        dist_cm = int(round(step_dist[i] * INV_STEP))
        if dist_cm <= 0:
            continue

        v_seg = np.empty(dist_cm, dtype=float)
        ang = slope_angle[i]
        rr_coef_i = step_rr[i]

        for ll in range(dist_cm):
            # 1) corner‐brake check
            reduce_lat = speed_reduction_caused_by_high_lat_acc(
                vx,
                ax_dec_latacc,
                v_max_latacc,
                dist_cm,
                steps,
                step_dist,
                ll,
                i,
            )
            # 2) crossroads / stop‐sign check
            red_status, v_target, ax_temp, stop_flag, _ = speed_reduction_caused_by_crossroads(
                vx,
                ax_dec_adapt,
                v_max_xroads,
                dist_cm,
                steps,
                step_dist,
                ll,
                i,
                v_x_total,
                v_seg[:ll],
            )

            # 3) decide power branch
            if red_status == 1 or stop_flag or reduce_lat == 1:
                # full braking
                p_in, p_roll, p_air, p_climb, p_acc = power_deceleration(
                    mass, rr_coef_i, ang, vx, cwxA, rho, p_flat, ax_temp
                )

            elif red_status == 2:
                # free‐rolling at give‐way
                ax_temp = power_input_off(mass, rr_coef_i, ang, vx, cwxA, rho)
                p_in = p_roll = p_air = p_climb = p_acc = 0.0

            else:
                # normal on‐power / free‐roll logic
                if ang <= alpha_vmax_ss:
                    # flat/downhill
                    if vx > v_max:
                        ax_temp = ax_dec_adapt
                        p_in = p_roll = p_air = p_climb = p_acc = 0.0

                    elif abs(vx - v_max) < 1e-12:
                        ax_temp = power_input_off(mass, rr_coef_i, ang, vx, cwxA, rho)
                        p_in = p_roll = p_air = p_climb = p_acc = 0.0

                    else:
                        ax_temp, p_in, p_roll, p_air, p_climb, p_acc = power_input_on(
                            mass, rr_coef_i, ang, vx, cwxA, rho, p_flat, v_max
                        )

                else:
                    # climbing steeper than free‐roll angle
                    if vx > v_max:
                        ax_temp = power_input_off(mass, rr_coef_i, ang, vx, cwxA, rho)
                        p_in = p_roll = p_air = p_climb = p_acc = 0.0
                    else:
                        ax_temp, p_in, p_roll, p_air, p_climb, p_acc = power_input_on(
                            mass, rr_coef_i, ang, vx, cwxA, rho, p_flat, v_max
                        )

            # 4) kinematics
            if ax_temp == 0.0:
                new_vx = vx
            else:
                new_vx_sq = vx * vx + 2.0 * ax_temp * STEP_SIZE
                new_vx = math.sqrt(new_vx_sq) if new_vx_sq > 0.0 else 0.0

            # 5) integrate energy & time
            if vx > 0.0:
                dt = STEP_SIZE / vx
                time_s       += dt
                energy       += p_in    * dt
                energy_roll  += p_roll  * dt
                energy_air   += p_air   * dt
                energy_climb += p_climb * dt
                energy_acc   += p_acc   * dt

            vx = new_vx
            v_seg[ll] = vx

        v_x_total.append(v_seg)

    # final aggregation
    v_all    = np.concatenate(v_x_total) if v_x_total else np.empty(0)
    distance = v_all.size * STEP_SIZE
    avg_speed = distance / time_s if time_s > 0.0 else 0.0

    return (
        energy,
        time_s,
        distance,
        avg_speed,
        energy_roll,
        energy_air,
        energy_climb,
        energy_acc,
    )


def bike_energy_model(
    cyclist_power: float,
    cyclist_mass:  float,
    rr_coef:       float,
    cwxA:          float,
    map_file:      Union[str, Path],
    temp_c:        float = 20.0,
) -> Tuple[float, float, float, float]:
    """
    High-level entrypoint, matching the old BikeEnergyModel signature.
    - loads GPX
    - computes air density ρ(T) = a·T² + b·T + c
    - computes the free-roll slope α_vmax_ss
    - calls simulate_energy and returns (E, T, D, Vavg)
    """
    # --- air density from config’s quadratic coefficients
    rho = AIR_DENSITY_A * temp_c**2 + AIR_DENSITY_B * temp_c + AIR_DENSITY_C

    # --- parse GPX & build step arrays
    (
        step_dist,
        step_ele,
        slope_angle,
        step_rr,
        _reduce_speed_dist,
        v_max_latacc,
        v_max_xroads,
    ) = map_data(map_file, rr_coef, AY_MAX, temp_c)

    if step_dist.size == 0:
        return 0.0, 0.0, 0.0, 0.0

    # --- free-rolling characteristic
    m_total = cyclist_mass + 18.3  # rider + bike
    alpha_vec, vx_vec = free_rolling_slope(m_total, step_rr[0], cwxA, rho)
    # pick the α closest to your target V_MAX
    idx = int(np.argmin(np.abs(vx_vec - V_MAX)))
    alpha_vmax_ss = alpha_vec[idx]

    # --- net power available on flat (subtract drivetrain loss)
    p_flat = cyclist_power - DRIVETRAIN_LOSS

    # --- run the core sim
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
        cwxA,
        rho,
        alpha_vmax_ss,
    )

    return E, T, D, Vavg
