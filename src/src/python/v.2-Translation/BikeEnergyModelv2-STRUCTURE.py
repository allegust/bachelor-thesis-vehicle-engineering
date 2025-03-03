#BikeEnergyModelv2.py
#This code has no finished functions
import math

# ------------------------------------------------------------------------
# PLACEHOLDER stubs for any missing sub-functions your Matlab script calls.
# You must fill these in or otherwise replace them with the real logic!
# ------------------------------------------------------------------------
def MapData_DD_Elbe(Map_Data_File, FigStatus, RRcoef_input, Ay_max, temp):
    """
    PLACEHOLDER function, must return:
      StepDist, StepElevation, StepAngle, StepRRcoef, ReduceSpeedDist,
      V_max_LatAcc, V_max_XRoads
    to mirror the Matlab version.
    """
    # TODO: Implement actual map-data import logic or custom code
    # that in Matlab was: [StepDist,StepElevation,StepAngle,StepRRcoef, ... ] = ...
    # Return dummy arrays for illustration (make them consistent lengths!)
    return [100]*10, [0]*10, [0]*10, [RRcoef_input]*10, [0]*10, [10]*10, [10]*10

def SpeedReductionCausedByHighLatAcc(vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                                     Dist, Steps, StepDist, ll, i):
    """
    PLACEHOLDER function, must return boolean or int (1 if reduce speed needed).
    In your Matlab code, the line is:
      ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(...)
    """
    # TODO: Implement your actual logic
    return 0

def SpeedReductionCausedByCrossRoads(vx, ax_Dec_adapt, V_max_XRoads,
                                     Dist, Steps, StepDist, ll, i,
                                     v_x_total, v_x):
    """
    PLACEHOLDER function. Must return:
      (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable)
    in line with the Matlab code:
      [ReduceSpeedStatus,v_x_target,ax,StopFlag,SpeedReductionTable] = SpeedReductionCausedByCrossRoads(...)
    """
    # TODO: Implement your actual logic
    ReduceSpeedStatus = 0
    v_x_target        = vx
    ax               = 0.0
    StopFlag         = 0
    SpeedReductionTable = 0
    return ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable

def PowerDeceleration(m, RRcoef, angle, vx, cwxA, rho, P, ax):
    """
    PLACEHOLDER function. Must return:
      (P_in, P_roll, P_air, P_climb, P_acc).
    In Matlab code: [P_in,P_roll,P_air,P_climb,P_acc] = PowerDeceleration(...)
    """
    # TODO: Implement your actual logic
    return 0.0, 0.0, 0.0, 0.0, 0.0

def PowerInputOff(m, RRcoef, angle, vx, cwxA, rho):
    """
    PLACEHOLDER function. Must return ax (acceleration).
    In Matlab code: ax = PowerInputOff(...)
    """
    # TODO: Implement your actual logic
    return 0.0

def PowerInputOn(m, RRcoef, angle, vx, cwxA, rho, P, V_max):
    """
    PLACEHOLDER function. Must return:
      (ax, P_in, P_roll, P_air, P_climb, P_acc)
    In Matlab code: [ax,P_in,P_roll,P_air,P_climb,P_acc] = PowerInputOn(...)
    """
    # TODO: Implement your actual logic
    ax = 0.0
    P_in = 0.0
    P_roll = 0.0
    P_air = 0.0
    P_climb = 0.0
    P_acc = 0.0
    return ax, P_in, P_roll, P_air, P_climb, P_acc

def FreeRollingSlope(m, RRcoef, cwxA, rho):
    """
    PLACEHOLDER function, must return (alpha_Vector, vx_Vector).
    In Matlab code: [alpha_Vector,vx_Vector] = FreeRollingSlope(m,RRcoef(1),cwxA,rho)
    """
    # Return dummy arrays of some length (e.g., 100) for illustration
    alpha_Vector = [0.0]*100
    vx_Vector    = [float(i)*0.1 for i in range(100)]
    return alpha_Vector, vx_Vector


# ------------------------------------------------------------------------
# MAIN FUNCTION: BikeEnergyModel
# This directly translates your Matlab function to Python
# ------------------------------------------------------------------------
def BikeEnergyModel(CyclistPowerIn, CyclistMassIn, CrIn, cwxA):
    """
    Translated from Matlab function:

    function [Energy,Time,Distance,AvgSpeed] = BikeEnergyModel(CyclistPowerIn,CyclistMassIn,CrIn,cwxA)

    The calculations that are made are kept intact wherever possible.
    """
    # Cyclist_Range = [10]    # in Matlab, some leftover, not used below
    Map_Data_File = 'GraphHopper_Track_DD_Koenigsbruecker_up'

    FigStatus = 0
    RRcoef_input = CrIn
    Ctrl_Figure = 0

    # set(0,'defaulttextInterpreter','latex') # (Matlab only) => not used in Python

    # Constants
    g = 9.81    # [m/s^2]
    R_earth = 6371e3
    r2d = 180.0/math.pi
    d2r = math.pi/180.0

    # Additional constants in the script
    temp = 20.0 # [degree Celsius]
    # rho = 10^(-5)*temp^2 - 0.0048*temp + 1.2926
    #   NOTE: In Python, ^ is bitwise-XOR. We must use ** for exponent.
    rho = (10**(-5))*(temp**2) - 0.0048*temp + 1.2926

    # Cyclist data
    V_max = 10.5      # [m/s]
    Ay_max = 2        # [m/s^2]
    ax_Dec_adapt = -0.3
    ax_Dec_LatAcc = -1.5

    # Import map data (using the selected function)
    (StepDist, StepElevation, StepAngle, StepRRcoef,
     ReduceSpeedDist, V_max_LatAcc, V_max_XRoads) = MapData_DD_Elbe(
        Map_Data_File, FigStatus, RRcoef_input, Ay_max, temp
    )

    # check length
    Steps = len(StepDist)
    if Steps != len(StepElevation):
        print(' Error in MapData function: Variable length does not agree ')
    elif Steps != len(StepRRcoef):
        print(' Error in MapData function: Variable length does not agree ')
    elif Steps != len(V_max_LatAcc):
        print(' Error in MapData function: Variable V_max_LatAcc length does not agree ')

    # define Cyclist mass and power
    m = CyclistMassIn + 18.3  # (In Matlab: CyclistMassIn + 18.3)
    # about 5 Watt lost in the bicycle drive train
    P = CyclistPowerIn - 5
    P_up = CyclistPowerIn*1.5 - 5
    V_max_LatAcc_individ = V_max_LatAcc

    # free rolling slope calculation
    alpha_Vector, vx_Vector = FreeRollingSlope(m, StepRRcoef[0], cwxA, rho)
    # find the index in vx_Vector where it equals V_max
    try:
        V_max_Index = vx_Vector.index(V_max)
        Alpha_Vmax_steadystate = alpha_Vector[V_max_Index]
    except ValueError:
        # fallback if V_max not found exactly in list
        # e.g. we can pick the nearest or do an interpolation
        Alpha_Vmax_steadystate = 0.0

    # Initialize accumulators
    Time           = 0.0
    Energy         = 0.0
    Energy_roll    = 0.0
    Energy_air     = 0.0
    Energy_climb   = 0.0
    Energy_acc     = 0.0

    v_x_total                = []
    Angle_total             = []
    Elevation_total         = []
    StopFlag_total          = []
    ReduceSpeedStatus_total = []
    SpeedReductionTable_total = []

    vx = 5.0   # start speed
    StopFlag = 0
    ax = 0.0

    # main loop over road segments
    for i in range(Steps):
        Dist = round(StepDist[i]/0.01)
        local_vx = vx   # in Matlab, v_x -> v_x(ll), but here we keep track in arrays
        Angle_ll = [StepAngle[i]]*Dist
        Elevation_total_ll = [StepElevation[i]]*Dist
        StopFlag_ll = [0]*Dist
        ReduceSpeedStatus_ll = [0]*Dist
        SpeedReductionTable_ll = [0]*Dist

        P_in_ll = [0.0]*Dist
        P_roll_ll = [0.0]*Dist
        P_air_ll  = [0.0]*Dist
        P_climb_ll = [0.0]*Dist
        P_acc_ll   = [0.0]*Dist

        for ll in range(Dist):
            # 1) check lat-acc-based slowdown
            ReduceSpeedFlag = SpeedReductionCausedByHighLatAcc(
                vx, ax_Dec_LatAcc, V_max_LatAcc_individ,
                Dist, Steps, StepDist, ll, i
            )

            # 2) check cross-roads-based slowdown/stop
            (ReduceSpeedStatus, v_x_target, ax, StopFlag, SpeedReductionTable) = \
                SpeedReductionCausedByCrossRoads(
                    vx, ax_Dec_adapt, V_max_XRoads,
                    Dist, Steps, StepDist, ll, i,
                    v_x_total, vx
                )

            brake = 0
            P_roll = 0.0
            P_air = 0.0
            P_climb = 0.0
            P_acc = 0.0
            P_in = 0.0

            if (ReduceSpeedStatus == 1 or StopFlag == 1):
                # Decelerate
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )

            elif ReduceSpeedFlag == 1:
                # decelerate with ax_Dec_LatAcc
                ax = ax_Dec_LatAcc
                (P_in, P_roll, P_air, P_climb, P_acc) = PowerDeceleration(
                    m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, ax
                )

            elif ReduceSpeedStatus == 2:
                # free rolling
                P_in = 0.0
                ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)

            else:
                # no forced slowdown, handle slope vs free rolling logic
                if StepAngle[i] <= Alpha_Vmax_steadystate:
                    # slope <= alpha for free rolling
                    if vx > V_max:
                        # must brake or do deceleration
                        brake = 1
                        ax = ax_Dec_adapt
                        P_in = 0.0
                    elif abs(vx - V_max) < 1e-9:
                        # exactly at V_max
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        # accelerate or approach new steady-state
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )
                else:
                    # slope > alpha for free rolling
                    if vx > V_max:
                        P_in = 0.0
                        ax = PowerInputOff(m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho)
                    else:
                        (ax, P_in, P_roll, P_air, P_climb, P_acc) = PowerInputOn(
                            m, StepRRcoef[i], StepAngle[i], vx, cwxA, rho, P, V_max
                        )

            # update speed from formula: vx^2 + 2 * ax * 0.01
            # careful if negative inside sqrt
            if (vx**2 + 2.0 * ax * 0.01) < 0:
                vx = 0.0
            else:
                vx = math.sqrt(vx**2 + 2.0 * ax * 0.01)

            if vx < 0.01 and StopFlag == 1:
                StopFlag = 0

            P_in_ll[ll]    = P_in
            P_roll_ll[ll]  = P_roll
            P_air_ll[ll]   = P_air
            P_climb_ll[ll] = P_climb
            P_acc_ll[ll]   = P_acc

            # accumulate energy/time
            if vx > 0:
                # 0.01 / vx => time interval for a 1 cm piece
                dt = 0.01 / vx
                Energy += P_in * dt
                Time   += dt
                Energy_roll  += P_roll  * dt
                Energy_air   += P_air   * dt
                Energy_climb += P_climb * dt
                Energy_acc   += P_acc   * dt

            StopFlag_ll[ll]          = StopFlag
            ReduceSpeedStatus_ll[ll] = ReduceSpeedStatus
            SpeedReductionTable_ll[ll] = SpeedReductionTable

        # append the step-by-step vectors for the segment
        v_x_total.extend([vx]*Dist)  # store final speeds at each sub-step
        Angle_total.extend(Angle_ll)
        Elevation_total.extend(Elevation_total_ll)
        StopFlag_total.extend(StopFlag_ll)
        ReduceSpeedStatus_total.extend(ReduceSpeedStatus_ll)
        SpeedReductionTable_total.extend(SpeedReductionTable_ll)

    Distance = len(v_x_total)/100.0
    AvgSpeed = Distance / Time if Time > 0 else 0.0

    # If you want the fraction
    # Energy_fraction = [Energy, Energy_roll, Energy_air, Energy_climb, Energy_acc]

    # Prepare outputs to match Matlab signature
    # function [Energy,Time,Distance,AvgSpeed] = ...
    return (Energy, Time, Distance, AvgSpeed)


# ---------------  Example usage if you wish to run this ---------------
# if __name__ == "__main__":
#     E, T, D, V = BikeEnergyModel(150, 70, 0.007, 0.45)
#     print("Energy:", E, "J")
#     print("Time:", T, "s")
#     print("Distance:", D, "m")
#     print("AvgSpeed:", V, "m/s =>", V*3.6, "km/h")
