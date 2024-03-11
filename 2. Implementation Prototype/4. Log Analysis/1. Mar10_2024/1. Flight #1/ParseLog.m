%script to parse the .bin ardupilot file into something that can be used by
%matlab

clear
clc
close all

%slice around the flight, LineNo is a parameter from the bin file, not the
%row number
lower_slice = 360917;
upper_slice = 395581;
slice_type = 'LineNo';

log = Ardupilog()
sliced_log = log.getSlice([lower_slice, upper_slice], slice_type).getStruct();
airspeed = sliced_log.ARSP.Airspeed;
thrust_pwm = sliced_log.RCOU.C3;
vtail_right_pwm = sliced_log.RCOU.C4;
vtail_left_pwm = sliced_log.RCOU.C2;

%convert avg vtail pwms to deflection angles
vtail_right_def = interp1([1810, 1150],[-20, 28], mean(vtail_right_pwm))
vtail_left_def = interp1([1800, 1100],[28, -20], mean(vtail_left_pwm))