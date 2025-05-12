# Bike Energy Model

This model simulates the energy consumption of cyclists over a given road segment using GPX track data. The simulation accounts for slope, wind resistance, rolling resistance, acceleration/deceleration, and environmental conditions. It is adapted from a MATLAB model, built by Malte Rothhämel, and fully rewritten and modularized in Python.

## Project Structure

```
├── bike_energy/                  # Core energy model package
│   ├── __init__.py               # Package initializer
│   ├── config.py                 # Loads and parses configuration from params.yaml
│   ├── controller.py             # Main entrypoint for GPX-based simulations
│   ├── map_data.py               # Parses GPX files and computes slope, curvature, etc.
│   ├── free_rolling.py           # Computes free-rolling slope-speed characteristics
│   ├── power_models.py           # Power input/output and acceleration models
│   ├── speed_control.py          # Speed reduction due to crossroads and cornering
│   └── simulator.py              # Core simulation loop
├── data/raw/                     # Folder to store GPX input files
├── params.yaml                   # Configuration for physical constants, cyclist data, etc.
├── pyproject.toml                # Python packaging metadata and dependencies
├── requirements.txt              # Python dependencies (e.g., numpy, scipy, gpxpy)
├── run.py                        # CLI runner for GPX-based simulations
├── optimeringsproblem v2-1.py    # Example optimization problem: over/under highway profiles
└── README.md                     # Project overview and usage
```

## How It Works

1. **Input**: A GPX file describing a cycling route.
2. **Map Processing**: Elevation and curvature data are extracted and converted to slope, step distances, etc.
3. **Simulation**: The cyclist model is simulated with power constraints, speed limits, and environmental factors.
4. **Output**: Energy, time, speed, and distance results for different cyclist types (women/men).

## Installation

```bash
pip install .
```

Or for development:

```bash
pip install -e .
```

Make sure your Python version is >= 3.8.

## ▶Running the Model

Use the CLI tool:

```bash
python run.py
```

To save the output as JSON:

```bash
python run.py -o output.json
```

## Output

The simulation prints and plots:
- Histograms and boxplots of energy, time, distance, and speed.
- Summary statistics for both women and men cyclist populations.

## Configuration

The simulation behavior is controlled via `params.yaml`. Here you can customize:
- Physical constants (e.g., gravity, air density)
- Cyclist VO2max data
- Weight distributions
- Rolling resistance and temperature effects
- Aerodynamic drag values
- Max slope clipping and GPX file path

## Example GPX File

Place your GPX file in the `data/raw/` directory and reference its name in `params.yaml`:
```yaml
map:
  gpx_folder: data/raw
  gpx_file: YourTrackFile.gpx
```

## 👥 Authors

Alexander Gustafsson & Eddie Tunas Ericson  
Vehicle Engineering Bachelor Thesis at KTH
