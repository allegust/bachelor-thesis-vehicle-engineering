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

# Import the updated BikeEnergyModel from BikeEnergyOptimized.py
from BikeEnergyModelFINAL import BikeEnergyModel
    # NUMBA_BikeEnergyModelOPT
    # BikeEnergyModelOPT
    # BikeEnergyModelFINAL
    # BikeEnergyModelOPTv2

def wblinv(percentiles, lambda_, k):
    """
    Replicates the Weibull inverse function used in MATLAB.
    """
    arr = []
    for p in percentiles:
        val = lambda_ * (-math.log(1.0 - p))**(1.0/k)
        arr.append(val)
    return np.array(arr)

def NormalDistributionWeight(MeanIn, Quantile25, Quantile75):
    """
    Replicates the NormalDistributionWeight function from the MATLAB code.
    """
    z25 = norm.ppf(0.25, loc=0, scale=1)
    z75 = norm.ppf(0.75, loc=0, scale=1)
    sigma = (Quantile75 - Quantile25) / (z75 - z25)

    percentiles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    results = []
    for p in percentiles:
        val = norm.ppf(p, loc=MeanIn, scale=sigma)
        results.append(val)
    return results

def EnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA):
    """
    A pass-through to BikeEnergyModel in BikeEnergyOptimized, using GPX_FILE_PATH
    for the route data. This matches the code references from your original logic.
    """
    return BikeEnergyModel(
        CyclistPowerIn,
        CyclistMassIn,
        CrIn,
        cwxA,
        map_file_path=GPX_FILE_PATH
    )

def combinations(CyclistPowerIn, CyclistMassIn, CrIn, cwxA_In):
    """
    Builds all 4D combinations of (Power, Mass, RollingResistance, cwxA).
    Mirroring your MATLAB approach with table2array in code.
    """
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
    Uses the updated BikeEnergyOptimized BikeEnergyModel function under the hood.
    """
    # Calculation of Percentiles
    percentiles = np.arange(0.1, 1.0, 0.1)

    # VO2max for women and men (mean over age from your references)
    VO2max_women = np.array([24.225, 27.6,   29.1,   30.975, 32.125,
                             33.625, 35.5,   37.8,   41.625])
    VO2max_men   = np.array([30.6,   33.2525,35.9125,37.8,   39.7,
                             41.55,  44.4875,46.525,49.125])

    PowerFactor = 71.8
    WeightSpecificPower_women = VO2max_women * PowerFactor / 1000.0
    WeightSpecificPower_men   = VO2max_men   * PowerFactor / 1000.0

    # Weighted means for women:
    Weight_mean_women = np.mean([61.8, 64.7, 67.3, 68.9, 70.3, 71.3, 72.3, 73.1])
    Weight_25_women   = np.mean([54.8, 57.3, 59.8, 61.1, 62.5, 63.5, 64.5, 65.2])
    Weight_75_women   = np.mean([68.5, 71.8, 74.6, 76.8, 78.6, 79.8, 80.8, 81.6])

    # Weighted means for men:
    Weight_mean_men   = np.mean([78.9, 81.6, 83.8, 85.6, 86.4, 86.6, 86.8, 86.8])
    Weight_25_men     = np.mean([69.5, 71.9, 73.8, 75.0, 75.8, 76.0, 76.2, 76.1])
    Weight_75_men     = np.mean([88.4, 91.6, 94.6, 96.8, 97.9, 98.3, 98.6, 98.7])

    # build deciles from NormalDistributionWeight
    Weight_deciles_women = NormalDistributionWeight(
        Weight_mean_women, Weight_25_women, Weight_75_women
    )
    Weight_deciles_men   = NormalDistributionWeight(
        Weight_mean_men,   Weight_25_men,   Weight_75_men
    )

    # Calculate the maximum power (W) from the VO2max and the weight data
    MaxPower_women = WeightSpecificPower_women * np.array(Weight_deciles_women)
    MaxPower_men   = WeightSpecificPower_men   * np.array(Weight_deciles_men)

    # Per your code, you use 0.3 of max:
    Power65_women = 0.3 * MaxPower_women
    Power65_men   = 0.3 * MaxPower_men

    # Rolling resistance from Weibull
    k = 2.28
    lambda_ = 0.00874
    CrIn = wblinv(percentiles, lambda_, k)

    # cwxA values
    cwxA_In = np.array([0.356, 0.402, 0.452, 0.493, 0.542,
                        0.588, 0.633, 0.695, 0.823])

    # We'll store final results in arrays
    Energy_women = []
    Time_women   = []
    Distance_women = []
    AvgSpeed_women = []

    Energy_men = []
    Time_men   = []
    Distance_men = []
    AvgSpeed_men = []

    # for s=1 => women, s=2 => men
    for s in [1, 2]:
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

        # replicate the loop from the old code: for row in CombinedInput
        # calling EnergyModel each time
        for row in CombinedInput:
            E, T, Dist, V = EnergyModel(row[0], row[1], row[2], row[3])
            E_array.append(E)
            T_array.append(T)
            D_array.append(Dist)
            V_array.append(V)

        # Sorting: in your old code, you sorted E_array, T_array, D_array, V_array
        # We'll do exactly that:
        E_array_sorted = sorted(E_array)
        T_array_sorted = sorted(T_array)
        D_array_sorted = sorted(D_array)
        V_array_sorted = sorted(V_array)

        # Then store in the final arrays
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

    # Print results (like your code does)
    print('Women: [', end='')
    for i in range(len(Energy_women)):
        print(f"({Energy_women[i]:.10f}, {Time_women[i]:.10f}, "
              f"{Distance_women[i]:.10f}, {AvgSpeed_women[i]:.10f})", end='')
        if i < len(Energy_women) - 1:
            print(', ', end='')
    print(']')

    print('Men: [', end='')
    for i in range(len(Energy_men)):
        print(f"({Energy_men[i]:.10f}, {Time_men[i]:.10f}, "
              f"{Distance_men[i]:.10f}, {AvgSpeed_men[i]:.10f})", end='')
        if i < len(Energy_men) - 1:
            print(', ', end='')
    print(']')

    # Return them in the same order as your original code
    return (Energy_women, Time_women, Distance_women, AvgSpeed_women,
            Energy_men,   Time_men,   Distance_men,   AvgSpeed_men)

def plot_same_as_matlab(AvgSpeed_kph, s):
    """
    Plots the same data as the MATLAB snippet:
      plot(AvgSpeed_kph, [1/length(AvgSpeed_kph) ... 1])
    X = speed in km/h (sorted)
    Y = fraction from 1/N up to 1
    s = 1 => 'women', s = 2 => 'men'
    """

    # 1) Sort the speeds
    speeds_sorted = np.sort(AvgSpeed_kph)

    # 2) Build the fraction array => 1..N => fraction from 1/N..1
    N = len(speeds_sorted)
    fractions = [(i + 1) / N for i in range(N)]

    # 3) Plot
    plt.plot(speeds_sorted, fractions, label='Cyclists')

    # 4) Match labels, style, etc.
    plt.grid(True)
    plt.xlabel('Average speed [km/h]')
    plt.ylabel('Sorted population of cyclists [1]')

    # 5) Optional: Add a text label for women or men in the same position as MATLAB
    if s == 1:
        plt.text(0.75, 0.12, 'women', transform=plt.gca().transAxes)
    else:
        plt.text(0.75, 0.12, 'men', transform=plt.gca().transAxes)

    plt.title('Distribution of Average Speeds over the Synthetic Population')
    plt.show()

def print_summary_stats(label, E, T, D, V):
    """
    Print min/median/mean/max for E, T, D, V.
    - E, T, D, V are Python lists or NumPy arrays
      (Energy [J], Time [s], Distance [m], Speed [m/s]).
    - label is "Women" or "Men".
    """
    E_a = np.array(E)
    T_a = np.array(T)
    D_a = np.array(D)
    V_a = np.array(V)  # speed in m/s

    print(f"\n=== Summary for {label} ===")

    # Speed in m/s
    min_v = V_a.min()
    med_v = np.median(V_a)
    mean_v = V_a.mean()
    max_v = V_a.max()
    print(f" Speed (m/s):    min={min_v:.2f}, median={med_v:.2f}, mean={mean_v:.2f}, max={max_v:.2f}")

    # Speed in km/h
    min_v_kmh   = min_v   * 3.6
    med_v_kmh   = med_v   * 3.6
    mean_v_kmh  = mean_v  * 3.6
    max_v_kmh   = max_v   * 3.6
    print(f" Speed (km/h):   min={min_v_kmh:.2f}, median={med_v_kmh:.2f}, mean={mean_v_kmh:.2f}, max={max_v_kmh:.2f}")

    # Energy
    min_e = E_a.min()
    med_e = np.median(E_a)
    mean_e = E_a.mean()
    max_e = E_a.max()
    print(f" Energy (J):     min={min_e:.2f}, median={med_e:.2f}, mean={mean_e:.2f}, max={max_e:.2f}")

    # Time
    min_t = T_a.min()
    med_t = np.median(T_a)
    mean_t = T_a.mean()
    max_t = T_a.max()
    print(f" Time (s):       min={min_t:.2f}, median={med_t:.2f}, mean={mean_t:.2f}, max={max_t:.2f}")

    # Distance
    min_d = D_a.min()
    med_d = np.median(D_a)
    mean_d = D_a.mean()
    max_d = D_a.max()
    print(f" Distance (m):   min={min_d:.2f}, median={med_d:.2f}, mean={mean_d:.2f}, max={max_d:.2f}")

    print("--------------------------------------------------\n")

if __name__ == "__main__":
    # 1) Run the full population simulation
    (E_w, T_w, D_w, V_w,
     E_m, T_m, D_m, V_m) = EnergyController()

    # 2) Print every result as (Energy, Time, Distance, AvgSpeed)
    print("\n=== Detailed results for Women ===")
    for i, (e, t, d, v) in enumerate(zip(E_w, T_w, D_w, V_w), start=1):
        print(f"{i:2d}: Energy={e:.2f} J, Time={t:.2f} s, Distance={d:.2f} m, AvgSpeed={v:.2f} m/s")
    print("\n=== Detailed results for Men   ===")
    for i, (e, t, d, v) in enumerate(zip(E_m, T_m, D_m, V_m), start=1):
        print(f"{i:2d}: Energy={e:.2f} J, Time={t:.2f} s, Distance={d:.2f} m, AvgSpeed={v:.2f} m/s")

    # 3) Print summary statistics
    import numpy as np

    def print_stats(label, E, T, D, V):
        E, T, D, V = map(np.array, (E, T, D, V))
        print(f"\n=== Summary for {label} ===")
        print(f" Speed (m/s):    min={V.min():.2f}, median={np.median(V):.2f}, mean={V.mean():.2f}, max={V.max():.2f}")
        print(f" Speed (km/h):   min={V.min()*3.6:.2f}, median={np.median(V)*3.6:.2f}, mean={V.mean()*3.6:.2f}, max={V.max()*3.6:.2f}")
        print(f" Energy (J):     min={E.min():.2f}, median={np.median(E):.2f}, mean={E.mean():.2f}, max={E.max():.2f}")
        print(f" Time (s):       min={T.min():.2f}, median={np.median(T):.2f}, mean={T.mean():.2f}, max={T.max():.2f}")
        print(f" Distance (m):   min={D.min():.2f}, median={np.median(D):.2f}, mean={D.mean():.2f}, max={D.max():.2f}")
        print("-" * 50)

    print_stats("Women", E_w, T_w, D_w, V_w)
    print_stats("Men",   E_m, T_m, D_m, V_m)

"""if __name__ == "__main__":
    E_w, T_w, D_w, V_w, E_m, T_m, D_m, V_m = EnergyController()

    # Convert speeds to km/h
    speeds_w_kmh = [v*3.6 for v in V_w]
    speeds_m_kmh = [v*3.6 for v in V_m]
    print(speeds_m_kmh)

    # Sort them
    speeds_w_kmh_sorted = sorted(speeds_w_kmh)
    speeds_m_kmh_sorted = sorted(speeds_m_kmh)
    print(speeds_m_kmh_sorted)

    # Create fraction (CDF) arrays for Y
    n_w = len(speeds_w_kmh_sorted)
    n_m = len(speeds_m_kmh_sorted)
    cdf_w = [(i+1)/n_w for i in range(n_w)]
    cdf_m = [(i+1)/n_m for i in range(n_m)]
    print(cdf_m)

    plt.figure(figsize=(7,5))
    plt.plot(speeds_w_kmh_sorted, cdf_w, label='Women')
    plt.plot(speeds_m_kmh_sorted, cdf_m, label='Men')
    plt.xlabel('Average speed [km/h]')
    plt.ylabel('Sorted population of cyclists [1]')
    plt.title('Distribution of Average Speeds Over the Synthetic Population')
    plt.grid(True)
    plt.legend()
    plt.show()

    # Compare summary stats
    print_summary_stats("Women", E_w, T_w, D_w, V_w)
    print_summary_stats("Men",   E_m, T_m, D_m, V_m)

    # Suppose these are the same speeds you got from your MATLAB computations
    # or from a Python "EnergyController" function. Just feed them in.
    for s in [1, 2]:
        some_avg_speeds_kmh = [19.2, 20.7, 21.5, 23.1, 24.0, 26.5, 27.2, 28.1, 30.4]
        plot_same_as_matlab(some_avg_speeds_kmh, s)
        """