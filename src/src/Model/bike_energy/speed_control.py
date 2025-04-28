from typing import Tuple, List, Any


def speed_reduction_high_lat_acc(
    vx: float,
    ax_dec_latacc: float,
    v_max_latacc: List[float],
    dist_cm: int,
    total_steps: int,
    step_dist: List[float],
    ll: int,
    i: int,
) -> bool:
    """
    Return True if upcoming curvature limit forces braking with ax_dec_latacc.
    """
    s_max = -0.5 * vx**2 / ax_dec_latacc
    Dist2End = [(dist_cm - ll) / 100.0]
    BrakeDist = [-(vx - v_max_latacc[i-1])**2 / (2 * ax_dec_latacc)]

    cnt = 0
    while max(Dist2End) < s_max and (i + cnt) < total_steps:
        Dist2End.append(Dist2End[cnt] + step_dist[i + cnt])
        BrakeDist.append(-(vx - v_max_latacc[i + cnt - 1])**2 / (2 * ax_dec_latacc))
        cnt += 1

    if vx > min(v_max_latacc[i-1:i+cnt]) and all(d > b for d, b in zip(Dist2End, BrakeDist)):
        return True
    return False


def speed_reduction_crossroads(
    vx: float,
    ax_dec_adapt: float,
    v_max_xroads: List[Tuple[float, Any]],
    total_cm: int,
    total_steps: int,
    step_dist: List[float],
    ll: int,
    i: int,
    v_x_total: List[List[float]],
    v_seg: List[float],
) -> Tuple[int, float, float, bool, int]:
    """
    Returns (status, v_target, ax, stop_flag, table_code):
      status: 0=none, 1=reduce to v_target, 2=free-roll
    """
    ax = ax_dec_adapt
    stop_flag = False

    dist_m = (sum(len(s) for s in v_x_total) + len(v_seg)) / 100.0
    idx = next((j for j,(d,_) in enumerate(v_max_xroads) if dist_m <= d), None)
    if idx is None:
        return 0, vx, ax, stop_flag, 0

    dist2 = v_max_xroads[idx][0] - dist_m
    code = v_max_xroads[idx][1]

    if code == 1 and dist2 < 20 and vx > 5:
        return 2, vx, 0.0, False, code

    targets = {2: 4.0, 3: 2.0, 4: 0.0}
    if code in targets:
        v_t = targets[code]
        bd = -(vx**2 - v_t**2) / (2 * -2.0)
        if vx > v_t and bd >= dist2:
            stop_flag = (code == 4)
            return 1, v_t, -2.0, stop_flag, code

    return 0, vx, ax, stop_flag, code
