%script to parse the .bin ardupilot file into something that can be used by
%matlab

clear
clc
close all

%slice around the flight, LineNo is a parameter from the bin file, not the
%row number
lower_slice = 514580; %change this
upper_slice = 530768; %change this
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

vtail_right_def = interp1([800, 2120],[24.5, -24], mean(vtail_right_pwm)); %1400 is trim
vtail_left_def = interp1([2200, 800],[-26, 23.5], mean(vtail_left_pwm)); %1510 is trim

%convert avg aileron pwms to deflection angles
aileron_right_def = interp1([2200, 1004],[16, -10], mean(aileron_right_pwm)) %1600 is trim
aileron_left_def = interp1([2000, 907],[-11.5, 15.5], mean(aileron_left_pwm)) %1400 is trim

%convert ruddervators to t-tail convention for simulation comparison
conversion_var = [1 1; -1 1]*[vtail_right_def; vtail_left_def];
elevator_deflection = conversion_var(1)
rudder_deflection = conversion_var(2)

%convert thrust pwm to percentage thrust
thrust_percentage = interp1([1100, 1900],[0, 100], mean(thrust_pwm)) %1400 is trim

mean(airspeed)
