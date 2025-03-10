import numpy as np

# Scaled segment distances (from MapData_Glado1.m)
Dist_step_met = np.array([
    24.65,
    2.40,
    14.55,
    60.05,
    49.67,
    103.7
]) * 100 / 35.2

# Elevation changes per segment
StepDeltaElevation = np.array([-1, 0, -3, 2, 7, -9])

# Cumulative elevations (initial elevation = 34 m)
StepElevation_init = 34
StepElevation = np.cumsum(np.insert(StepDeltaElevation, 0, StepElevation_init))[:-1]

# Slope angles (radians), limited to ±0.2 rad
StepAngle = np.arctan(StepDeltaElevation / Dist_step_met)
StepAngle = np.clip(StepAngle, -0.2, 0.2)

# Rolling resistance (use default calculation from the .m file)
temp = 20
C_r = 0.274 / (temp + 46.8) + 0.004
RRcoef = C_r * np.ones(len(Dist_step_met))

# Reduce speed flags (not used anymore, set dummy values)
ReduceSpeedDist = np.zeros(len(Dist_step_met))

# V_max_LatAcc (given Ay_max example and radius)
Ay_max = np.array([2, 2, 2])  # example limits
R = np.array([100, 3, 50, 100, 100, 100])
V_max_LatAcc = np.sqrt(Ay_max[:, None] * R)
