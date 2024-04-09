%script to parse the .bin ardupilot file into something that can be used by
%matlab

clear
clc
close all

%slice around the flight, LineNo is a parameter from the bin file, not the
%row number

%these slice numbers have been updated for tthe thrid level section of the
%flight in this directory
lower_slice = 570359; 
upper_slice = 582599; 
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

%% Plotting the elevator and thrust real and simulation

%preparing the time data
time_slice = sliced_log.ATT.TimeS;
time_slice(end)=[];
strt = time_slice(1);
time_corrected = [];
for i = time_slice
    time_corrected=[time_corrected,i-strt];
end

%preparing the thrust values
percent_thrust_array = [];
for i = thrust_pwm
    percent_thrust_array = [percent_thrust_array,interp1([1100, 1900],[0, 100], i)];
end

%creating the elevator deflection array
%convert avg aileron pwms to deflection angles
VT_r = [];
VT_l = [];

%left
for i = aileron_left_pwm
    VT_l = [VT_l,interp1([2200, 800],[-26, 23.5], i)];
end

% right
for i = aileron_right_pwm
    VT_r = [VT_r,interp1([800, 2120],[24.5, -24], i)];
end

%convert ruddervators to t-tail convention for simulation comparison
ind = linspace(1,length(VT_r),length(VT_l));
elv_ar = [];
for i = ind
    conversion_var_ar = [1 1; -1 1]*[VT_r(i); VT_l(i)];
    elv_ar = [elv_ar,conversion_var_ar(1)];
end

%defining the collected simulation data TODO
load('data_elevator.mat')
load('data_throttle.mat')

%plotting trust

figure
ax = gca;
set(gca,'FontSize',12)
plot(time_corrected,percent_thrust_array,'LineWidth',2,'Color',[220/255 117/255 0])
hold on
plot(thrtl_dt.Time(119:end)-8,thrtl_dt.Data(119:end),'LineWidth',2,'Color',[0 0 1])
legend({'Flight Data', 'Our Control System'},'Location','best')
xlabel('Time (s)')
ylabel('% Throttle')
grid on
ax = gca;
ax.GridAlpha = 0.55;  % [R, G, B]
title('Simulation vs. Flight Throttle Comparison')
hold off
%plotting elevator

figure
ax = gca;
set(gca,'FontSize',12)
plot(time_corrected,elv_ar,'LineWidth',2,'Color',[220/255 117/255 0])
hold on
plot(elvtr_dt.Time(119:end)-8,elvtr_dt.Data(119:end),'LineWidth',2,'Color',[0 0 1])
legend({'Flight Data', 'Our Control System'},'Location','best')
xlabel('Time (s)')
ylabel('Elevator angle (Degrees)')
grid on
ax = gca;
ax.GridAlpha = 0.55;  % [R, G, B]
title('Simulation vs. Flight Elevator Comparison')
hold off
%% graphing alt and speed for poster

%getting real data
time_alt = sliced_log.GPS.TimeS-strt;
altitude_real = sliced_log.GPS.Alt-1196.71;
speed_time = sliced_log.ARSP.TimeS-strt;

length(altitude_real)
length(airspeed)
length(speed_time)

%importing sim data from files
load("data_alt.mat")
load("speed.mat")

%plotting alt
figure
ax = gca;
set(gca,'FontSize',12)
plot(time_alt,altitude_real,'LineWidth',2,'Color',[220/255 117/255 0])
hold on 
plot(altitude.Time(119:end)-8,altitude.Data(119:end),'LineWidth',2,'Color',[0 0 1])
legend({'Flight Data', 'Our Control System'},'Location','best')
xlabel('Time (s)')
ylabel('Altitude relative to the ground (m)')
grid on
ax = gca;
ax.GridAlpha = 0.55;  % [R, G, B]
title('Simulation vs. Flight Altitude Comparison')
hold off

%plotting speed
figure
ax = gca;
plot(speed_time,smooth(airspeed),'LineWidth',2,'Color',[220/255 117/255 0])
hold on 
plot(speed.Time(119:end)-8,speed.Data(119:end),'LineWidth',2,'Color',[0 0 1])
legend({'Flight Data', 'Our Control System'},'Location','best')
xlabel('Time (s)')
ylabel('Airspeed (m/s)')
title('Simulation vs. Flight Airspeed Comparison')
grid on
ax = gca;
ax.GridAlpha = 0.55;  % [R, G, B]
set(gca,'FontSize',12)
hold off