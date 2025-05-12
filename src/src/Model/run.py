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

    # Extract sorted arrays
    Energy_women = results["energy_women"]
    Time_women = results["time_women"]
    Distance_women = results["distance_women"]
    AvgSpeed_women = results["avg_speed_women"]

    Energy_men = results["energy_men"]
    Time_men = results["time_men"]
    Distance_men = results["distance_men"]
    AvgSpeed_men = results["avg_speed_men"]

    # Print exactly like ControllerOPT.py
    print('Women: [', end='')
    for i in range(len(Energy_women)):
        print(f"({Energy_women[i]:.10f}, {Time_women[i]:.10f}, {Distance_women[i]:.10f}, {AvgSpeed_women[i]:.10f})", end='')
        if i < len(Energy_women) - 1:
            print(', ', end='')
    print(']')

    print('Men: [', end='')
    for i in range(len(Energy_men)):
        print(f"({Energy_men[i]:.10f}, {Time_men[i]:.10f}, {Distance_men[i]:.10f}, {AvgSpeed_men[i]:.10f})", end='')
        if i < len(Energy_men) - 1:
            print(', ', end='')
    print(']')

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