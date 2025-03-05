#BikeEnergyControllerv2.py
import os
import math
import numpy as np
import scipy
from scipy.stats import norm

# 1) find_repo_root code
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
GPX_FILE_NAME = "MapData_NewLocation.gpx"
GPX_FILE_PATH = os.path.join(DATA_DIR, GPX_FILE_NAME)

if not os.path.exists(GPX_FILE_PATH):
    print(f"ERROR: GPX file not found at {GPX_FILE_PATH}")
    print(f"Check that the file is in the correct location: {DATA_DIR}")
    exit(1)

# 2) import the updated BikeEnergyModel
from BikeEnergyOptimized import BikeEnergyModel
#BikeEnergyModelv3 

def wblinv(percentiles, lambda_, k):
    """
    Equivalent of Matlab wblinv(percentiles, lambda, k).
    The Weibull inverse CDF:
        x = lambda * (-ln(1 - p))^(1/k)
    """
    # handle array input
    arr = []
    for p in percentiles:
        val = lambda_ * (-math.log(1.0 - p))**(1.0/k)
        arr.append(val)
    return np.array(arr)

def NormalDistributionWeight(MeanIn, Quantile25, Quantile75):
    """
    Equivalent of the Matlab subfunction in BikeEnergyController:
    function [percentile_values] = NormalDistributionWeight(MeanIn,Quantile25,Quantile75)

    We define an approach to norminv( p, mean, sigma ) ~ the inverse CDF of normal distribution.
    In Python, we can use 'scipy.stats.norm.ppf' if allowed, but
    to keep dependencies minimal, we do a manual approach or approximate. 
    For brevity, let's do an approximate approach with ppf from scipy if available.
    """

    # replicate logic from Matlab:
    # z25 = norminv(0.25, 0, 1)
    # z75 = norminv(0.75, 0, 1)
    z25 = norm.ppf(0.25, loc=0, scale=1)
    z75 = norm.ppf(0.75, loc=0, scale=1)
    sigma = (Quantile75 - Quantile25)/(z75 - z25)

    percentiles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    results = []
    for p in percentiles:
        val = norm.ppf(p, loc=MeanIn, scale=sigma)
        results.append(val)
    return results

# If you keep BikeEnergyModel in a separate file, you can do:
# from BikeEnergyModel import BikeEnergyModel
#
# Otherwise, define it inline or rename the call below:
def EnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA):
    """
    If in your original second script you used EnergyModel(...),
    we simply forward to BikeEnergyModel(...).
    BUT now we also pass GPX_FILE_PATH so it uses the correct file.
    """
    # Pass the GPX file path from the top of this file
    return BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, map_file_path=GPX_FILE_PATH)

def combinations(CyclistPowerIn, CyclistMassIn, CrIn, cwxA_In):
    """
    Replacement for the Matlab 'combinations()' or code that does
    'table2array(CombinedInput)'. 
    In Matlab, you are enumerating all combinations of inputs across those deciles.

    E.g.:
    CombinedInput = all possible combos of
        CyclistPowerIn[i], CyclistMassIn[j], CrIn[k], cwxA_In[l]

    Then we store them in a structure. 
    Below is a minimal example enumerating every combination.
    """
    import itertools
    # Each is presumably 1D arrays or lists
    all_combos = []
    for p in CyclistPowerIn:
        for m in CyclistMassIn:
            for cr in CrIn:
                for cwx in cwxA_In:
                    all_combos.append( (p, m, cr, cwx) )
    # Return something akin to a Nx4 array:
    return np.array(all_combos)


def EnergyController():
    """
    Translated from the Matlab script:
    function [Energy_women,Time_women,Distance_women,AvgSpeed_women,
              Energy_men,Time_men,Distance_men,AvgSpeed_men] = BikeEnergyController()
    """
    Ctrl_Save = 1

    # set(0,'defaulttextInterpreter','latex') # (Matlab only) => not used in Python

    # Calculation of Percentiles
    percentiles = np.arange(0.1, 1.0, 0.1)  # 0.1:0.1:0.9


    # VO2max for men/women
    VO2max_women = np.array([24.225, 27.6, 29.1, 30.975, 32.125, 33.625, 35.5, 37.8, 41.625])
    VO2max_men   = np.array([30.6,   33.2525,35.9125,37.8,   39.7,   41.55, 44.4875,46.525,49.125])

    PowerFactor = 71.8
    # weight specific power
    WeightSpecificPower_women = VO2max_women * PowerFactor / 1000.0
    WeightSpecificPower_men   = VO2max_men   * PowerFactor / 1000.0

    Weight_mean_women = np.mean([61.8, 64.7, 67.3, 68.9, 70.3, 71.3, 72.3, 73.1])
    Weight_25_women   = np.mean([54.8, 57.3, 59.8, 61.1, 62.5, 63.5, 64.5, 65.2])
    Weight_75_women   = np.mean([68.5, 71.8, 74.6, 76.8, 78.6, 79.8, 80.8, 81.6])

    Weight_mean_men   = np.mean([78.9, 81.6, 83.8, 85.6, 86.4, 86.6, 86.8, 86.8])
    Weight_25_men     = np.mean([69.5, 71.9, 73.8, 75.0, 75.8, 76.0, 76.2, 76.1])
    Weight_75_men     = np.mean([88.4, 91.6, 94.6, 96.8, 97.9, 98.3, 98.6, 98.7])

    # compute distribution
    Weight_deciles_women = NormalDistributionWeight(Weight_mean_women, Weight_25_women, Weight_75_women)
    Weight_deciles_men   = NormalDistributionWeight(Weight_mean_men,   Weight_25_men,   Weight_75_men)

    # now compute max power
    MaxPower_women = WeightSpecificPower_women * np.array(Weight_deciles_women)
    MaxPower_men   = WeightSpecificPower_men   * np.array(Weight_deciles_men)

    # 65% of max
    Power65_women = 0.3 * MaxPower_women
    Power65_men   = 0.3 * MaxPower_men   # in your code it is 0.3, but text says 65% ???

    # input distribution for rolling resistance Cr => Weibull
    k = 2.28
    lambda_ = 0.00874
    CrIn = wblinv(percentiles, lambda_, k)

    # cwxA
    cwxA_In = np.array([0.356, 0.402, 0.452, 0.493, 0.542, 0.588, 0.633, 0.695, 0.823])

    # We will store the output
    Energy_women = []
    Time_women = []
    Distance_women = []
    AvgSpeed_women = []

    Energy_men = []
    Time_men = []
    Distance_men = []
    AvgSpeed_men = []

    # main loop
    for s in [1, 2]:
        if s == 1:
            CyclistPowerIn = Power65_women
            CyclistMassIn  = Weight_deciles_women
        else:
            CyclistPowerIn = Power65_men
            CyclistMassIn  = Weight_deciles_men

        # Build all combos
        CombinedInput = combinations(CyclistPowerIn, CyclistMassIn, CrIn, cwxA_In)

        # prepare arrays to store results
        E_array = []
        T_array = []
        D_array = []
        V_array = []

        ct = 1
        for row in CombinedInput:
            # row = (p_in, m_in, cr_in, cwx_in)
            # pass it to the model
            E, T, D, V = EnergyModel(row[0], row[1], row[2], row[3])
            E_array.append(E)
            T_array.append(T)
            D_array.append(D)
            V_array.append(V)
            ct += 1

        # sort them
        E_array_sorted = sorted(E_array)
        T_array_sorted = sorted(T_array)
        D_array_sorted = sorted(D_array)
        V_array_sorted = sorted(V_array)

        # store into correct arrays
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

    # Print results in a similar format to Python
    # Women
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
    EnergyController()
# --------------- Example usage ---------------
#if __name__ == "__main__":
#    (Ew,Tw,Dw,Aw, Em,Tm,Dm,Am) = EnergyController()
#    # Do something with these results
#    pass
