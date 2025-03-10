function [Dist_step_met,StepElevation,StepAngle,RRcoef,ReduceSpeedDist,V_max_LatAcc,V_max_XRoads] = MapData_Glado1(Map_Data_File,FigStatus,RRcoef_input,Ay_max,temp)
%Special MapData file for Glado existing cycleway 
    %
    % Input:
    %
    %   Map_Data_File is a file name of the data file to be found in C:\Users\Malte\Documents\03_Projects\30_BicycleEnergyModell\GPS_Data
    %   FigStatus     is a flag whether to plot figures (=1) or not (=0)
    %   RRcoef_input  is a rolling resistance coefficient that is assumed over the whole map file (if nothing else is available)
    %       in case of RRcoef_input == 0, a coefficient of 0.08 is assumed
    %   Ay_max        is a vector containing 3 maximum lateral acceleration limits for different cyclists
    %   temp          is the temperature in degrees
    %
    % Outputs:
    %
    %   Dist_step_met   is a vector containing the distances in meter of each road piece 
    %   StepElevation   is a vector containing the elevation in meter of each road piece 
    %   StepAngle       is a vector containing the slope angle of each road piece 
    %   RRcoef          is a vector containing the rolling resistance coefficient of each road piece 
    %   ReduceSpeedDist is a vector containing a flag for need to reduce speed for each road piece
    %
    % Author: Malte Rothhämel.
    % Version: special 1.0 (March 2022)

%% Constants
R_earth=6371*10^3;   % earth radius [m]
r2d = 180/pi;
d2r = pi/180;

%% Manually extracted data from "Gladö Fastställelsehandling" and "Illustrationsplaner"
% cycleway
Dist_step_met = [
                24.65
                 2.40
                14.55
                60.05
                49.67
                103.7];

Dist_step_met = Dist_step_met * 100 / 35.2;     % Scale correction from map
           
StepDeltaElevation = [
                -1
                0
                -3
                2
                7
                -9];

StepElevation_init = 34;
StepElevation = StepElevation_init + StepDeltaElevation(1);
for i = 2:length(StepDeltaElevation)
    StepElevation(i) = StepElevation(i-1) + StepDeltaElevation(i);
end %for i

R = [
    100
    3	
    50	
    100
    100
    100];

%% Calculating the difference in height from point to point
StepAngle = atan(StepDeltaElevation./Dist_step_met');  % [rad]
% limit angle to +/-0.2
StepAngle(StepAngle>0.2) = 0.2;
StepAngle(StepAngle<-0.2) = -0.2;

%% Rolling resistance
% if nothing else is known about Rolling Resistance
if RRcoef_input ~= 0
    RRcoef = RRcoef_input * ones(1,length(Dist_step_met));
else
    C_r = 0.274/(temp + 46.8);      % according to findings in CD7 (chequer plate at 300kPa inflation pressure)
    C_r = C_r + 0.004;              % this is a non-specified correction value
    % the CD7 experiments were performed with a low RR tyre and on steel
    % surface. Therefore, here an offset is added to become more realistic
    RRcoef = C_r * ones(1,length(Dist_step_met));
end % if RRcoef_input 

%% Reduce speed
% not used any more
% fill with dummy
        ReduceSpeedDist = 0;

%% Calculate V_max_LatAcc

% Ay_max is a vector with 3 limits for 3 cyclists
V_max_LatAcc = sqrt(Ay_max.*R);       % a_y = v^2 / r   <=>  v^2 = a_y * r

%% Configure the needs for deceleration caused by cross-roads
% Assumptions: cross-roads with
% Status 1: priority-to-the-right-rule, no traffic expected good sight -> free rolling at v_x
% Status 2: with expected road-users who have right-of-way or who do not respect the priority -> brake to 4m/s
% Status 3: give way rule with bad sight, cycle barrier -> brake to 2m/s
% Status 4: Stop-sign / red traffic lights -> brake to 0m/s
V_max_XRoads = [0,0]; % default if there are no data available

  % Distance [m], Status

end %function
