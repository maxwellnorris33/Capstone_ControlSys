%script to parse the .bin ardupilot file into something that can be used by
%matlab

clear
clc
close all

%slice around the flight, LineNo is a parameter from the bin file, not the
%row number
lower_slice = 362609;
upper_slice = 390725;
slice_type = 'LineNo';

log = Ardupilog()
sliced_log = log.getSlice([lower_slice, upper_slice], slice_type).getStruct();
airspeed = sliced_log.ARSP.Airspeed;
thrust_pwm = sliced_log.RCOU.C3;
vtail_right_pwm = sliced_log.RCOU.C4;
vtail_left_pwm = sliced_log.RCOU.C2;
aileron_right_pwm = sliced_log.RCOU.C1;
aileron_left_pwm = sliced_log.RCOU.C5;

%convert avg vtail pwms to deflection angles

%note, actual deflections are estimated at the moment, will need actual
%measurements to map PWM to. Right now the mapping indicates that trim is
%not 0 deflection.

vtail_right_def = interp1([1780, 1155],[28, -20], mean(vtail_right_pwm)) %1400 is trim
vtail_left_def = interp1([1805, 1138],[-20, 28], mean(vtail_left_pwm)) %1510 is trim

%convert avg aileron pwms to deflection angles

%no data for mapping yet, need max min pwm and corresponding deflections
aileron_right_def = interp1([1695, 1398],[28, -20], mean(aileron_right_pwm)) %1600 is trim
aileron_left_def = interp1([1695, 1398],[28, -20], mean(aileron_left_pwm)) %1400 is trim
