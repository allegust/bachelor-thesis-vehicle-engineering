# bike_energy/config.py

import yaml
from pathlib import Path
from typing import Any, Dict, List, Union

_cfg: Dict[str, Any] = None

def load_config(path: str = "params.yaml") -> Dict[str, Any]:
    global _cfg
    if _cfg is None:
        _cfg = yaml.safe_load(Path(path).read_text())
    return _cfg

"""
BASE = Path(__file__).parent
def load_config(fname="params.yaml"):
    return yaml.safe_load((BASE / fname).read_text())"""

# load once
_config = load_config()

#──────────────────────────────────────────────────────────────────────────────
# Physical constants (physical_constants)
#──────────────────────────────────────────────────────────────────────────────
_PHYS = _config["physical_constants"]
GRAVITY: float       = _PHYS["gravity"]
EARTH_RADIUS: float  = _PHYS["earth_radius"]
AIR_DENSITY_A: float = float(_PHYS["air_density_coefficients"]["a"])
AIR_DENSITY_B: float = float(_PHYS["air_density_coefficients"]["b"])
AIR_DENSITY_C: float = float(_PHYS["air_density_coefficients"]["c"])

"""AIR_DENSITY_A: float = _PHYS["air_density_coefficients"]["a"]
AIR_DENSITY_B: float = _PHYS["air_density_coefficients"]["b"]
AIR_DENSITY_C: float = _PHYS["air_density_coefficients"]["c"]"""

#──────────────────────────────────────────────────────────────────────────────
# Simulation parameters (simulation)
#──────────────────────────────────────────────────────────────────────────────
_SIM = _config["simulation"]
STEP_SIZE: float = float(_SIM["step_size"])
#STEP_SIZE: float = _SIM["step_size"]
V_MAX: float     = _SIM["v_max"]
AY_MAX: float    = _SIM["ay_max"]

AX_ADAPT:  float = _SIM["ax"]["adapt"]
AX_LATACC: float = _SIM["ax"]["latacc"]
AX_STOP:   float = _SIM["ax"]["stop"]

#──────────────────────────────────────────────────────────────────────────────
# Power model (power_model)
#──────────────────────────────────────────────────────────────────────────────
_PM = _config["power_model"]
VO2_DECILES_WOMEN: List[float] = _PM["vo2max"]["women"]
VO2_DECILES_MEN:   List[float] = _PM["vo2max"]["men"]
POWER_FACTOR:      float        = _PM["power_factor"]
DRIVETRAIN_LOSS:   float        = _PM["drivetrain_loss"]

#──────────────────────────────────────────────────────────────────────────────
# Weight model (weight_model)
#──────────────────────────────────────────────────────────────────────────────
_WM = _config["weight_model"]
WEIGHT_PERCENTILES:   List[float] = _WM["percentiles"]
WEIGHT_MEAN_WOMEN:     List[float] = _WM["women"]["mean_values"]
WEIGHT_Q25_WOMEN:      List[float] = _WM["women"]["quantile25_values"]
WEIGHT_Q75_WOMEN:      List[float] = _WM["women"]["quantile75_values"]
WEIGHT_MEAN_MEN:       List[float] = _WM["men"]["mean_values"]
WEIGHT_Q25_MEN:        List[float] = _WM["men"]["quantile25_values"]
WEIGHT_Q75_MEN:        List[float] = _WM["men"]["quantile75_values"]

#──────────────────────────────────────────────────────────────────────────────
# Rolling resistance (rolling_resistance)
#──────────────────────────────────────────────────────────────────────────────
_RR = _config["rolling_resistance"]
RR_PERCENTILES:      List[float] = _RR["percentiles"]
RR_WEIBULL_K:         float       = _RR["weibull"]["k"]
RR_WEIBULL_LAMBDA:    float       = _RR["weibull"]["lambda"]
RR_TEMP_LOOKUP:       List[float] = _RR.get("temp_lookup", [])
RR_BASE_NUMERATOR:    float       = _RR["base"]["numerator"]
RR_BASE_TEMP_OFFSET:  float       = _RR["base"]["temp_offset"]
RR_BASE_OFFSET:       float       = _RR["base"]["offset"]

#──────────────────────────────────────────────────────────────────────────────
# Aerodynamic drag (aerodynamic_drag)
#──────────────────────────────────────────────────────────────────────────────
_AD = _config["aerodynamic_drag"]
CWXA_VALUES: List[float] = _AD["cwxA_values"]

#──────────────────────────────────────────────────────────────────────────────
# Map parameters (map)
#──────────────────────────────────────────────────────────────────────────────
_MAP = _config["map"]
GPX_FOLDER: str = _MAP["gpx_folder"]
GPX_FILE:   str = _MAP["gpx_file"]
MAX_SLOPE:  float = _MAP["max_slope"]
