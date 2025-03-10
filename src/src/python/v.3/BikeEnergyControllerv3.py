# BikeEnergyControllerv2.py

import os
import math
import numpy as np
import scipy
from scipy.stats import norm
import matplotlib.pyplot as plt

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

if not os.path.exists(GPX_FILE_PATH):
    print(f"ERROR: GPX file not found at {GPX_FILE_PATH}")
    print(f"Check that the file is in the correct location: {DATA_DIR}")
    exit(1)

# 2) import the updated BikeEnergyModel
from BikeEnergyOptimized import BikeEnergyModel
# (Your script references "BikeEnergyModelv3" or "BikeEnergyOptimized";

def wblinv(percentiles, lambda_, k):
    arr = []
    for p in percentiles:
        val = lambda_ * (-math.log(1.0 - p))**(1.0/k)
        arr.append(val)
    return np.array(arr)

def NormalDistributionWeight(MeanIn, Quantile25, Quantile75):
    z25 = norm.ppf(0.25, loc=0, scale=1)
    z75 = norm.ppf(0.75, loc=0, scale=1)
    sigma = (Quantile75 - Quantile25)/(z75 - z25)

    percentiles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    results = []
    for p in percentiles:
        val = norm.ppf(p, loc=MeanIn, scale=sigma)
        results.append(val)
    return results

def EnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA):
    """
    A simple pass-through to BikeEnergyModel, 
    including the path to the current GPX file.
    """
    return BikeEnergyModel(
        CyclistPowerIn,
        CyclistMassIn,
        CrIn,
        cwxA,
        map_file_path=GPX_FILE_PATH
    )

def combinations(CyclistPowerIn, CyclistMassIn, CrIn, cwxA_In):
    import itertools
    all_combos = []
    for p in CyclistPowerIn:
        for m in CyclistMassIn:
            for cr in CrIn:
                for cwx in cwxA_In:
                    all_combos.append((p, m, cr, cwx))
    return np.array(all_combos)

def EnergyController():
    """
    Translated from the Matlab script BikeEnergyController but in Python form.
    """
    # Calculation of Percentiles
    percentiles = np.arange(0.1, 1.0, 0.1)

    # VO2max for men/women
    VO2max_women = np.array([24.225, 27.6, 29.1, 30.975, 32.125, 33.625, 35.5, 37.8, 41.625])
    VO2max_men   = np.array([30.6,   33.2525,35.9125,37.8,   39.7,   41.55, 44.4875,46.525,49.125])

    PowerFactor = 71.8
    WeightSpecificPower_women = VO2max_women * PowerFactor / 1000.0
    WeightSpecificPower_men   = VO2max_men   * PowerFactor / 1000.0

    Weight_mean_women = np.mean([61.8, 64.7, 67.3, 68.9, 70.3, 71.3, 72.3, 73.1])
    Weight_25_women   = np.mean([54.8, 57.3, 59.8, 61.1, 62.5, 63.5, 64.5, 65.2])
    Weight_75_women   = np.mean([68.5, 71.8, 74.6, 76.8, 78.6, 79.8, 80.8, 81.6])

    Weight_mean_men   = np.mean([78.9, 81.6, 83.8, 85.6, 86.4, 86.6, 86.8, 86.8])
    Weight_25_men     = np.mean([69.5, 71.9, 73.8, 75.0, 75.8, 76.0, 76.2, 76.1])
    Weight_75_men     = np.mean([88.4, 91.6, 94.6, 96.8, 97.9, 98.3, 98.6, 98.7])

    Weight_deciles_women = NormalDistributionWeight(Weight_mean_women, Weight_25_women, Weight_75_women)
    Weight_deciles_men   = NormalDistributionWeight(Weight_mean_men,   Weight_25_men,   Weight_75_men)

    MaxPower_women = WeightSpecificPower_women * np.array(Weight_deciles_women)
    MaxPower_men   = WeightSpecificPower_men   * np.array(Weight_deciles_men)

    # typically 65% of max used in your script, but you have 0.3 => consistent with your code
    Power65_women = 0.3 * MaxPower_women
    Power65_men   = 0.3 * MaxPower_men

    # Rolling resistance from a Weibull distribution
    k = 2.28
    lambda_ = 0.00874
    CrIn = wblinv(percentiles, lambda_, k)

    # cwxA array
    cwxA_In = np.array([0.356, 0.402, 0.452, 0.493, 0.542, 0.588, 0.633, 0.695, 0.823])

    # We'll store final results
    Energy_women = []
    Time_women = []
    Distance_women = []
    AvgSpeed_women = []

    Energy_men = []
    Time_men = []
    Distance_men = []
    AvgSpeed_men = []

    for s in [1,2]:
        if s == 1:
            CyclistPowerIn = Power65_women
            CyclistMassIn  = Weight_deciles_women
        else:
            CyclistPowerIn = Power65_men
            CyclistMassIn  = Weight_deciles_men

        # build all combos
        CombinedInput = combinations(CyclistPowerIn, CyclistMassIn, CrIn, cwxA_In)

        E_array = []
        T_array = []
        D_array = []
        V_array = []

        for row in CombinedInput:
            E, T, D, V = EnergyModel(row[0], row[1], row[2], row[3])
            E_array.append(E)
            T_array.append(T)
            D_array.append(D)
            V_array.append(V)

        E_array_sorted = sorted(E_array)
        T_array_sorted = sorted(T_array)
        D_array_sorted = sorted(D_array)
        V_array_sorted = sorted(V_array)

        if s == 1:
            Energy_women   = E_array_sorted
            Time_women     = T_array_sorted
            Distance_women = D_array_sorted
            AvgSpeed_women = V_array_sorted
        else:
            Energy_men   = E_array_sorted
            Time_men     = T_array_sorted
            Distance_men = D_array_sorted
            AvgSpeed_men = V_array_sorted

    # Print results
    print('Women: [', end='')
    for i in range(len(Energy_women)):
        print(f"({Energy_women[i]:.10f}, {Time_women[i]:.10f}, "
              f"{Distance_women[i]:.10f}, {AvgSpeed_women[i]:.10f})", 
              end='')
        if i < len(Energy_women)-1:
            print(', ', end='')
    print(']')

    print('Men: [', end='')
    for i in range(len(Energy_men)):
        print(f"({Energy_men[i]:.10f}, {Time_men[i]:.10f}, "
              f"{Distance_men[i]:.10f}, {AvgSpeed_men[i]:.10f})",
              end='')
        if i < len(Energy_men)-1:
            print(', ', end='')
    print(']')

    return (Energy_women,Time_women,Distance_women,AvgSpeed_women,
            Energy_men,Time_men,Distance_men,AvgSpeed_men)


if __name__ == "__main__":
    # Run the main controller
    (Ew,Tw,Dw,Aw, Em,Tm,Dm,Am) = EnergyController()

    # Now let's produce a distribution plot "like in Matlab"
    # We assume Aw and Am are already sorted ascending. 
    # Convert from m/s to km/h
    Aw_kph = [v * 3.6 for v in Aw]
    Am_kph = [v * 3.6 for v in Am]

    # We'll create a fraction array from 1/len(...) to 1 in increments of 1/len(...)
    if len(Aw_kph) > 0:
        y_women = np.arange(1, len(Aw_kph)+1) / float(len(Aw_kph))
    else:
        y_women = []

    if len(Am_kph) > 0:
        y_men = np.arange(1, len(Am_kph)+1) / float(len(Am_kph))
    else:
        y_men = []

    plt.figure()
    # Plot women's distribution
    if len(Aw_kph) > 0:
        plt.plot(Aw_kph, y_women, label='Women')
    # Plot men's distribution
    if len(Am_kph) > 0:
        plt.plot(Am_kph, y_men, label='Men')

    plt.xlabel('Average speed [km/h]')
    plt.ylabel('Cumulative fraction [1]')
    plt.title('Distribution of Average Speeds')
    plt.grid(True)
    plt.legend()
    plt.show()
