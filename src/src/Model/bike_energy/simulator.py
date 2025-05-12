import math
from pathlib import Path
from typing import Tuple, Union

import numpy as np

from bike_energy.config import (
    GRAVITY,
    STEP_SIZE,
    V_MAX,
    AY_MAX,
    AX_ADAPT,
    AX_LATACC,
    DRIVETRAIN_LOSS,
    AIR_DENSITY_A,
    AIR_DENSITY_B,
    AIR_DENSITY_C
)
from bike_energy.map_data import map_data
from bike_energy.free_rolling import free_rolling_slope
from bike_energy.power_models import (
    power_input_on,
    power_input_off,
    power_deceleration,
)
from bike_energy.speed_control import (
    speed_reduction_caused_by_crossRoads,
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

    time_s = energy = energy_roll = energy_air = energy_climb = energy_acc = 0.0
    vx = 5.0  # initial speed [m/s]
    v_x_total = []

    steps = step_dist.size

    for i in range(steps):
        dist_cm = int(round(step_dist[i] * INV_STEP))
        if dist_cm <= 0:
            continue

        v_seg = np.empty(dist_cm, dtype=float)
        ang = slope_angle[i]
        rr_coef_i = step_rr[i]

        for ll in range(dist_cm):
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

            red_status, v_target, ax_temp, stop_flag, _ = speed_reduction_caused_by_crossRoads(
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

            if red_status == 1 or stop_flag or reduce_lat == 1:
                p_in, p_roll, p_air, p_climb, p_acc = power_deceleration(
                    mass, rr_coef_i, ang, vx, cwxA, rho, p_flat, ax_temp
                )
            elif red_status == 2:
                ax_temp = power_input_off(mass, rr_coef_i, ang, vx, cwxA, rho)
                p_in = p_roll = p_air = p_climb = p_acc = 0.0
            else:
                if ang <= alpha_vmax_ss:
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
                    if vx > v_max:
                        ax_temp = power_input_off(mass, rr_coef_i, ang, vx, cwxA, rho)
                        p_in = p_roll = p_air = p_climb = p_acc = 0.0
                    else:
                        ax_temp, p_in, p_roll, p_air, p_climb, p_acc = power_input_on(
                            mass, rr_coef_i, ang, vx, cwxA, rho, p_flat, v_max
                        )

            new_vx_sq = vx * vx + 2.0 * ax_temp * STEP_SIZE
            vx = math.sqrt(max(new_vx_sq, 0.0))

            if vx > 0.0:
                dt = STEP_SIZE / vx
                time_s += dt
                energy += p_in * dt
                energy_roll += p_roll * dt
                energy_air += p_air * dt
                energy_climb += p_climb * dt
                energy_acc += p_acc * dt

            v_seg[ll] = vx

        v_x_total.append(v_seg)

    v_all = np.concatenate(v_x_total) if v_x_total else np.empty(0)
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
