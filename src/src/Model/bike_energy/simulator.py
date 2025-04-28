from pathlib import Path
from typing import Tuple

import numpy as np

from bike_energy.config import Params
from bike_energy.map_data import map_data
from bike_energy.free_rolling import free_rolling_slope
from bike_energy.power_models import (
    power_input_on, power_input_off, power_deceleration
)
from bike_energy.speed_control import (
    speed_reduction_high_lat_acc, speed_reduction_crossroads
)


def simulate_energy(
    params: Params,
    cyclist_mass: float,
    cyclist_power: float,
    cr: float,
    cwxA: float,
) -> Tuple[float, float, float, float]:
    """
    Run the full centimetre-by-centimetre simulation for one rider:
    returns (Energy[J], Time[s], Distance[m], AvgSpeed[m/s]).
    """
    sim = params.simulation
    phys = params.physical_constants
    map_cfg = params.map

    # load map arrays
    map_path = Path(map_cfg.gpx_folder) / map_cfg.gpx_file
    step_dist, step_ele, slope_ang, step_rr, reduce_x, v_max_latacc, v_max_xroads = map_data(
        map_path,
        cr,
        sim.ay_max,
        sim.temperature,
        phys.earth_radius,
        map_cfg.max_slope,
    )

    # free-rolling slope
    rho = (
        sim.temperature**2 * phys.air_density_coefficients.a
        + sim.temperature  * phys.air_density_coefficients.b
        + phys.air_density_coefficients.c
    )
    alpha_vec, vx_vec = free_rolling_slope(
        cyclist_mass + 18.3,
        step_rr[0],
        cwxA,
        rho,
        phys.gravity
    )
    idx_vmax = min(range(len(vx_vec)), key=lambda j: abs(vx_vec[j] - sim.v_max))
    alpha_ss = alpha_vec[idx_vmax]

    STEP = sim.step_size
    v_x_total = []
    vx = 5.0
    E = T = 0.0

    for i in range(len(step_dist)):
        dist_cm = int(round(step_dist[i] / STEP))
        rr = step_rr[i]
        ang = slope_ang[i]

        seg_vx = []
        for ll in range(dist_cm):
            slow_lat = speed_reduction_high_lat_acc(
                vx, sim.ax.latacc, v_max_latacc,
                dist_cm, len(step_dist), step_dist, ll, i
            )
            status, target, ax_cr, stop, _ = speed_reduction_crossroads(
                vx, sim.ax.adapt, v_max_xroads,
                dist_cm, len(step_dist), step_dist, ll, i,
                v_x_total, seg_vx
            )

            if status == 1 or stop:
                P_in, Pr, Pa, Pc, Pa2 = power_deceleration(
                    cyclist_mass + 18.3, rr, ang, vx,
                    cwxA, rho,
                    cyclist_power - params.power_model.drivetrain_loss,
                    ax_cr, phys.gravity
                )
                ax = ax_cr
            elif slow_lat:
                ax = sim.ax.latacc
                P_in, Pr, Pa, Pc, Pa2 = power_deceleration(
                    cyclist_mass + 18.3, rr, ang, vx,
                    cwxA, rho,
                    cyclist_power - params.power_model.drivetrain_loss,
                    ax_cr, phys.gravity
                )
            elif status == 2:
                ax = power_input_off(
                    cyclist_mass + 18.3, rr, ang, vx,
                    cwxA, rho, phys.gravity
                )
                P_in = Pr = Pa = Pc = Pa2 = 0.0
            else:
                if ang <= alpha_ss:
                    if vx > sim.v_max:
                        ax = sim.ax.adapt
                        P_in = Pr = Pa = Pc = Pa2 = 0.0
                    else:
                        ax, P_in, Pr, Pa, Pc, Pa2 = power_input_on(
                            cyclist_mass + 18.3, rr, ang, vx, cwxA,
                            rho,
                            cyclist_power - params.power_model.drivetrain_loss,
                            sim.v_max, phys.gravity
                        )
                else:
                    if vx > sim.v_max:
                        ax = power_input_off(
                            cyclist_mass + 18.3, rr, ang, vx,
                            cwxA, rho, phys.gravity
                        )
                        P_in = Pr = Pa = Pc = Pa2 = 0.0
                    else:
                        ax, P_in, Pr, Pa, Pc, Pa2 = power_input_on(
                            cyclist_mass + 18.3, rr, ang, vx, cwxA,
                            rho,
                            cyclist_power - params.power_model.drivetrain_loss,
                            sim.v_max, phys.gravity
                        )

            new_v2 = vx*vx + 2*ax*STEP
            new_v = np.sqrt(new_v2) if new_v2 > 0 else 0.0
            if new_v > 0:
                dt = STEP / new_v
                T += dt
                E += P_in * dt

            vx = new_v
            seg_vx.append(vx)

        v_x_total.append(seg_vx)

    distance = sum(len(s) for s in v_x_total) * STEP
    avg_speed = distance / T if T > 0 else 0.0
    return E, T, distance, avg_speed
