# params_opt.py

"""
Parameters for run_optimize.py
"""

# ─── Geometry ────────────────────────────────────────────────────────────────
highway_width = 55.0       # [m]
max_slope    = 0.05        # ±5 %
start_height = 2.0         # [m]
end_height   = -1.0        # [m]

min_height_over  =  5.0   # the peak you climb to above the highway
min_height_under = -3.0   # the dip you descend to below the highway

# Discretization for the curves and flats
n_curve = 120
n_flat  =  60

# ─── GPS anchor for synthetic coords ────────────────────────────────────────
gps_anchor = {
    "lat0": 59.329,        # degrees
    "lon0": 18.068,
}

# ─── Rider & bike model ─────────────────────────────────────────────────────
power = 150.0              # W
mass  = 70.0               # kg
c_r   = 0.007              # rolling-resistance coefficient
cwxA  = 0.45               # aerodynamic drag area (Cw·A)
