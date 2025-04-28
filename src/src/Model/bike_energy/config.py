import yaml
from pathlib import Path
from typing import List, Union
from pydantic import BaseModel, Field


class AirDensityCoefficients(BaseModel):
    a: float
    b: float
    c: float


class PhysicalConstants(BaseModel):
    gravity: float
    earth_radius: int
    air_density_coefficients: AirDensityCoefficients


class AxParams(BaseModel):
    adapt: float
    latacc: float
    stop: float


class SimulationParams(BaseModel):
    step_size: float
    v_max: float
    ay_max: float
    ax: AxParams


class Vo2MaxParams(BaseModel):
    women: List[float]
    men: List[float]


class PowerModelParams(BaseModel):
    vo2max: Vo2MaxParams
    power_factor: float
    drivetrain_loss: float


class WeightStats(BaseModel):
    mean_values: List[float]
    quantile25_values: List[float]
    quantile75_values: List[float]


class WeightModelParams(BaseModel):
    percentiles: List[float]
    women: WeightStats
    men: WeightStats


class WeibullParams(BaseModel):
    k: float
    lambda_: float = Field(..., alias="lambda")


class BaseRollingResistanceParams(BaseModel):
    numerator: float
    temp_offset: float
    offset: float


class RollingResistanceParams(BaseModel):
    percentiles: List[float]
    weibull: WeibullParams
    temp_lookup: List[float]
    base: BaseRollingResistanceParams


class AerodynamicDragParams(BaseModel):
    cwxA_values: List[float]


class MapParams(BaseModel):
    gpx_folder: str
    gpx_file: str
    max_slope: float


class Params(BaseModel):
    physical_constants: PhysicalConstants
    simulation: SimulationParams
    power_model: PowerModelParams
    weight_model: WeightModelParams
    rolling_resistance: RollingResistanceParams
    aerodynamic_drag: AerodynamicDragParams
    map: MapParams


def load_params(path: Union[str, Path] = "params.yaml") -> Params:
    """
    Load simulation parameters from a YAML file and return a Params object.

    Args:
        path: Path to the YAML parameter file (default: 'params.yaml').

    Returns:
        Params: Parsed parameters as a Pydantic model.
    """
    yaml_path = Path(path)
    data = yaml.safe_load(yaml_path.read_text())
    # allow alias for 'lambda'
    return Params.parse_obj(data)


# Example usage:
# params = load_params("params.yaml")
# print(params.simulation.step_size)
