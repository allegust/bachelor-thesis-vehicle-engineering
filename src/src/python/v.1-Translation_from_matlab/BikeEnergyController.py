import os
import numpy as np
from BikeEnergyModel import bike_energy_model




# Dynamisk sökväg till GPX-filer baserat på repository-root

def find_repo_root():
    current_dir = os.path.abspath(os.path.dirname(__file__))
    while current_dir != "/" and not os.path.exists(os.path.join(current_dir, ".git")):
        current_dir = os.path.dirname(current_dir)
    return current_dir if os.path.exists(os.path.join(current_dir, ".git")) else None

# Automatiskt hitta repository-roten
REPO_ROOT = find_repo_root()
if REPO_ROOT is None:
    print("ERROR: Could not determine repository root. Make sure you're inside the repo.")
    exit(1)

# Dynamisk sökväg till data/raw/ oavsett vem som kör koden
DATA_DIR = os.path.join(REPO_ROOT, "data/data/raw")

# Byt GPX-fil enkelt genom att ändra denna variabel
GPX_FILE_NAME = "Sockenplan_Huddinge_MinaKartaLantmateri.gpx"
GPX_FILE_PATH = os.path.join(DATA_DIR, GPX_FILE_NAME)

# 🛠 Kontrollera att GPX-filen finns
if not os.path.exists(GPX_FILE_PATH):
    print(f"ERROR: GPX file not found at {GPX_FILE_PATH}")
    print(f"Check that the file is in the correct location: {DATA_DIR}")
    exit(1)




def bike_energy_controller():
    percentiles = np.linspace(0.1, 0.9, 9)
    power_factor = 71.8  # Conversion factor [W/(l/min VO2)]

    vo2max_women = np.array([24.225, 27.6, 29.1, 30.975, 32.125, 33.625, 35.5, 37.8, 41.625])
    vo2max_men = np.array([30.6, 33.2525, 35.9125, 37.8, 39.7, 41.55, 44.4875, 46.525, 49.125])
    
    weight_women = np.mean([61.8, 64.7, 67.3, 68.9, 70.3, 71.3, 72.3, 73.1])
    weight_men = np.mean([78.9, 81.6, 83.8, 85.6, 86.4, 86.6, 86.8, 86.8])
    
    power_women = (vo2max_women * power_factor / 1000) * weight_women
    power_men = (vo2max_men * power_factor / 1000) * weight_men

    # FIX: Pass GPX file path to `bike_energy_model()`
    results_women = [bike_energy_model(p, weight_women, 0.007, 0.45, GPX_FILE_PATH) for p in power_women]
    results_men = [bike_energy_model(p, weight_men, 0.007, 0.45, GPX_FILE_PATH) for p in power_men]

    return results_women, results_men

if __name__ == "__main__":
    print(f"Using GPX file: {GPX_FILE_PATH}")
    results_women, results_men = bike_energy_controller()
    print("Women:", results_women)
    print("Men:", results_men)