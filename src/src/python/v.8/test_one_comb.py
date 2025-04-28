from BikeEnergyModelFINAL import BikeEnergyModel
import os

# One test case
P_test    = 200   # W
m_test    = 75.0    # kg
Cr_test   = 0.007   # rolling resistance
cwxA_test = 0.45    # cw*A

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

E, T, Dist, Vavg = BikeEnergyModel(
    P_test, m_test, Cr_test, cwxA_test,
    GPX_FILE_PATH
)

print(f"Energy = {E:.1f} J")
print(f"Time   = {T:.1f} s")
print(f"Distance = {Dist:.1f} m")
print(f"Avg speed = {Vavg*3.6:.2f} km/h")
