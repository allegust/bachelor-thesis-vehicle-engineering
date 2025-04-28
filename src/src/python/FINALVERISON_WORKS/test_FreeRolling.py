# test_one_comb.py

import numpy as np
import os
from FreeRollingSlope import Free_Rolling_Slope
from BikeEnergyModelFINAL import BikeEnergyModel

#--- 1) Define your test inputs (same as MATLAB) ---
P_test    = 200.0   # W
m_test    = 75.0    # kg
Cr_test   = 0.007   # rolling resistance
cwxA_test = 0.45    # cw*A

# match the rho formula in your model
temp = 20.0
rho = (10**-5) * temp**2 - 0.0048 * temp + 1.2926

def find_repo_root():
    current_dir = os.path.abspath(os.path.dirname(__file__))
    while current_dir != "/" and not os.path.exists(os.path.join(current_dir, ".git")):
        current_dir = os.path.dirname(current_dir)
    return current_dir if os.path.exists(os.path.join(current_dir, ".git")) else None

REPO_ROOT = find_repo_root()
if REPO_ROOT is None:
    print("ERROR: Could not determine repository root. Make sure you're inside the repo.")
    exit(1)

DATA_DIR = os.path.join(REPO_ROOT, "data", "data", "raw")
GPX_FILE_NAME = "GraphHopper_Track_DD_Koenigsbruecker_up.gpx"
GPX_FILE_PATH = os.path.join(DATA_DIR, GPX_FILE_NAME)


#--- 2) Compute and print Python's alpha_vmax_ss ---
alpha_vec, vx_vec = Free_Rolling_Slope(m_test, Cr_test, cwxA_test, rho)

# force an exact 0.1-step grid so 10.5 is in there
vx_vec = np.linspace(0.1, 20.0, int((20.0 - 0.1)/0.1) + 1)
idx = np.argmin(np.abs(vx_vec - 10.5))

print(f"Python alpha_vmax_ss = {alpha_vec[idx]:.8f} rad")

#--- 3) Now run the full model once ---
E, T, Dist, Vavg, eq_count = BikeEnergyModel(
    P_test, m_test, Cr_test, cwxA_test,
    map_file_path="path/to/track.gpx"
)

print(f"Energy = {E:.1f} J")
print(f"Time   = {T:.1f} s")
print(f"Distance = {Dist:.1f} m")
print(f"Avg speed = {Vavg*3.6:.2f} km/h")
print(f"Python eq_count (|vx - V_max|<tol hits) = {eq_count}")
