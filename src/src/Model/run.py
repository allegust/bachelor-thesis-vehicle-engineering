#!/usr/bin/env python3
"""
run.py

Entry‐point for the Bike Energy Model simulation.

Usage:
  ./run.py            # uses params.yaml, prints JSON to stdout
  ./run.py -o out.json  # save results to out.json
"""

import argparse
import json
import numpy as np
from pathlib import Path

from bike_energy.controller import EnergyController


def parse_args():
    p = argparse.ArgumentParser(description="Bike Energy Model Simulation")
    p.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Write simulation results to JSON file"
    )
    return p.parse_args()


def summarize(label, values):
    values = np.array(values)
    print(f"{label:<15} min={values.min():.2f}, median={np.median(values):.2f}, mean={values.mean():.2f}, max={values.max():.2f}")

def print_summary(group_name, energy, time, distance, speed):
    print(f"\n=== Summary for {group_name} ===")
    summarize("Speed (m/s)", speed)
    summarize("Speed (km/h)", np.array(speed) * 3.6)
    summarize("Energy (J)", energy)
    summarize("Time (s)", time)
    summarize("Distance (m)", distance)
    print("="*47)

def main():
    args = parse_args()

    try:
        results = EnergyController()
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise

    if args.output:
        try:
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(results, f, indent=2)
            print(f"Results written to {args.output}")
        except Exception as e:
            print(f"Failed to write results: {e}")

    # Extract sorted arrays
    Energy_women = results["energy_women"]
    Time_women = results["time_women"]
    Distance_women = results["distance_women"]
    AvgSpeed_women = results["avg_speed_women"]

    Energy_men = results["energy_men"]
    Time_men = results["time_men"]
    Distance_men = results["distance_men"]
    AvgSpeed_men = results["avg_speed_men"]

    # Print clean summary
    print_summary("Women", Energy_women, Time_women, Distance_women, AvgSpeed_women)
    print_summary("Men", Energy_men, Time_men, Distance_men, AvgSpeed_men)

if __name__ == "__main__":
    main()


"""
def parse_args():
    p = argparse.ArgumentParser(description="Bike Energy Model Simulation")
    p.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Write simulation results to JSON file"
    )
    return p.parse_args()

def main():
    args = parse_args()

    try:
        results = EnergyController()
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise

    # Save to JSON if output path is given
    if args.output:
        try:
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(results, f, indent=2)
            print(f"Results written to {args.output}")
        except Exception as e:
            print(f"Failed to write results: {e}")

    # Always print summary stats and show plots
    import numpy as np
    import matplotlib.pyplot as plt

    def summarize(label, values):
        print(f"\n--- {label} ---")
        print(f"Mean: {np.mean(values):.2f}")
        print(f"Min:  {np.min(values):.2f}")
        print(f"Max:  {np.max(values):.2f}")

    for key, values in results.items():
        summarize(key, values)

    # Histograms
    def show_histogram(metric, xlabel):
        plt.hist(results[f"{metric}_women"], bins=10, alpha=0.6, label="Women")
        plt.hist(results[f"{metric}_men"], bins=10, alpha=0.6, label="Men")
        plt.xlabel(xlabel)
        plt.ylabel("Count")
        plt.title(f"Distribution of {xlabel}")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    show_histogram("avg_speed", "Average Speed (m/s)")
    show_histogram("energy", "Energy (J)")
    show_histogram("time", "Time (s)")
    show_histogram("distance", "Distance (m)")

    # Boxplots
    def show_boxplot(metric, ylabel):
        plt.boxplot(
            [results[f"{metric}_women"], results[f"{metric}_men"]],
            labels=["Women", "Men"]
        )
        plt.title(f"{ylabel} Distribution")
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.show()

    show_boxplot("energy", "Energy (J)")
    show_boxplot("avg_speed", "Average Speed (m/s)")
    show_boxplot("time", "Time (s)")
    show_boxplot("distance", "Distance (m)")


def main():
    args = parse_args()

    try:
        results = EnergyController()
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise

    if args.output:
        try:
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(results, f, indent=2)
            print(f"Results written to {args.output}")
        except Exception as e:
            print(f"Failed to write results: {e}")
    else:
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
"""