#!/usr/bin/env python3
"""
run.py

Entry‐point for the Bike Energy Model simulation.

Usage:
  ./run.py            # uses params.yaml, prints JSON to stdout
  ./run.py -c my.yaml # use a different config file
  ./run.py -o out.json  # save results to out.json
"""             # python run.py -c params.yaml -o out.json


import argparse
import json
from pathlib import Path

from bike_energy.config import load_params
from bike_energy.controller import EnergyController

def parse_args():
    p = argparse.ArgumentParser(description="Bike Energy Model Simulation")
    p.add_argument(
        "-c", "--config",
        type=Path,
        default=Path("params.yaml"),
        help="YAML config file (default: params.yaml)"
    )
    p.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Write simulation results to JSON file"
    )
    return p.parse_args()

def main():
    args = parse_args()

    # 1) Load all parameters
    params = load_params(args.config)

    # 2) Run controller and get back a dict-like result
    #    e.g. {
    #      "energy_women": [...],
    #      "time_women":   [...],
    #      ...
    #    }
    results = EnergyController(params)

    # 3) Serialize to JSON
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        print(f"Results written to {args.output}")
    else:
        print(json.dumps(results, indent=2))

if __name__ == "__main__":
    main()
