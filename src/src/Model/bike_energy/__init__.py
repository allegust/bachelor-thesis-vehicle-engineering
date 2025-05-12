"""
bike_energy: Simulate cyclist energy consumption on road segments.
"""

__version__ = "1.0.0"
__author__ = "Alexander Gustafsson & Eddie Tunas Ericson"

from .config import load_config
from .controller import EnergyController
from .simulator import simulate_energy
from .map_data import map_data
from .free_rolling import free_rolling_slope
from .speed_control import speed_reduction_caused_by_crossRoads, speed_reduction_caused_by_high_lat_acc
from .power_models import power_input_on, power_input_off, power_deceleration

__all__ = [
    "load_config",
    "EnergyController",
    "simulate_energy",
    "map_data",
    "free_rolling_slope",
    "speed_reduction_caused_by_crossRoads",
    "speed_reduction_caused_by_high_lat_acc",
    "power_input_on",
    "power_input_off",
    "power_deceleration",
]
