clc
close all
clear all
log = Ardupilog('March16_FL1_SLF.bin');

trimPitchAngle = 0.013367; % From Matlab Trimming Script in Capstone Files

%DEFINING START AND END OF LOG SLICE IN SECONDS%

startTime = 4070;
endTime = 4094;

sliced_log = log.getSlice([startTime,endTime],'TimeS');

%AIRSPEED DATA AND PLOT%

airspeed = sliced_log.ARSP.Airspeed;
arsptime = sliced_log.ARSP.TimeS - startTime;


plot(arsptime, airspeed)
title("U (Airspeed)")

%U V AND W PLOTS FROM ACCELEROMETER - RESULTS IN QUESTION%

figure

uacc = sliced_log.IMU.AccX;
vacc = sliced_log.IMU.AccZ;
wacc = sliced_log.IMU.AccY;

IMUtime = sliced_log.IMU.TimeS-startTime;

uvel = cumtrapz(uacc,IMUtime) + airspeed(1,1);
vvel = cumtrapz(vacc,IMUtime) - airspeed(1,1)*tan(trimPitchAngle);
wvel = cumtrapz(wacc,IMUtime);

subplot(3,3,1)
plot(IMUtime,uvel)
title("U (Longitudinal Velocity) - IMU")

subplot(3,3,2)
plot(IMUtime,vvel)
title("V (Vertical Velocity) - IMU")

subplot(3,3,3)
plot(IMUtime,wvel)
title("W (Lateral Velocity) - IMU")

%ANGULAR RATE VARIABLES P Q AND R%

p = sliced_log.IMU.GyrX;

subplot(3,3,4)
plot(IMUtime,deg2rad(p))
title("p (Roll Rate)")

r = sliced_log.IMU.GyrZ;

subplot(3,3,6)
plot(IMUtime,deg2rad(r))
title("r (Yaw Rate)")

q = sliced_log.IMU.GyrY;

subplot(3,3,5)
plot(IMUtime,deg2rad(q))
title("q (Pitch Rate)")

%ATTITUDE VARIABLES PHI(Pitch), THETA(Roll), and PSI(Yaw)%

pitchangle = sliced_log.ATT.Pitch;
rollangle = sliced_log.ATT.Roll;
yawangle = sliced_log.ATT.Yaw;

ATTtime = sliced_log.ATT.TimeS - startTime;

subplot(3,3,8)
plot(ATTtime, deg2rad(pitchangle))
title("Theta (Pitch Angle)")

subplot(3,3,7)
plot(ATTtime,deg2rad(rollangle))
title("Phi (Roll Angle)")

subplot(3,3,9)
plot(ATTtime,deg2rad(yawangle))
title("Psi (Yaw Angle)")

%INITIAL CONDITION CALC AND OUTPUT%

disp("u = " + airspeed(1,1))
disp("v = " + -airspeed(1,1)*tan(trimPitchAngle))
disp("w = " + 0)
disp("p = " + p(1,1))
disp("q = " + q(1,1))
disp("r = " + r(1,1))
disp("phi = " + rollangle(1,1))
disp("theta = " + pitchangle(1,1))
disp("psi = " + yawangle(1,1))

%RC OUTPUTS%

figure

vtail_right_pwm = sliced_log.RCOU.C4;
vtail_left_pwm = sliced_log.RCOU.C2;
aileron_right_pwm = sliced_log.RCOU.C1;
aileron_left_pwm = sliced_log.RCOU.C5;
Throttle = sliced_log.RCOU.C3;

vtail_right_def = interp1([1150, 1810],[24.5, -24], (vtail_right_pwm)); %1400 is trim
vtail_left_def = interp1([1100, 1800],[-26, 23.5], (vtail_left_pwm)); %1510 is trim
aileron_right_def = interp1([2000, 907],[16, -10], (aileron_right_pwm));%1600 is trim
aileron_left_def = interp1([2200, 1004],[-11.5, 15.5], (aileron_left_pwm)); %1400 is trim
throttle_def = interp1([1300, 1680], [0, 1], (Throttle));
% Splitting the Data sets
% This has to be case specific, taking data from the trim values to assume
% the elevator defelction then calculating the yaw as a result
% Trim data for 20 m/s run
u0 = [-9.25638059468980e-08; 0.0378095450195116; -3.36838234336176e-07; 0.385300921860774;];
% Manual intervention to determine the continued input
% Aileron is at zero so need to correct, subtract the intial average value
% from the rest of the data to get the correction, makes the simulation
% consistent

aileron_right_def_start = deg2rad(mean(aileron_right_def(1:100)));
aileron_left_def_start = deg2rad(mean(aileron_left_def(1:100)));

Adjust_aileron_right_def = deg2rad(aileron_right_def) - aileron_right_def_start;
Adjust_aileron_left_def = deg2rad(aileron_right_def) - aileron_left_def_start;
for i = 1:length(Adjust_aileron_right_def)
    Aileron_Data(i) = mean([Adjust_aileron_right_def(i),Adjust_aileron_left_def(i)]);
end
Aileron_Data = Aileron_Data';

% Splitting the vtail using the trimed states from the u0 function, since
% the ardupilot system uses thrust to increase alitutde
InitialElevator_Left = deg2rad(vtail_left_def(1));
InitialElevator_Right = deg2rad(vtail_right_def(1));

% First Attempt, taking avereage of vtail to use as the rudder input,
% keeping elevator at trim
for y = 1:length(vtail_right_def)
    Rudder_Data(y) = mean([vtail_right_def(y),vtail_left_def(y)]);
end
Rudder_Data = Rudder_Data';

% Second Attempt using formula tail = (yaw+ pitch)*0.5
for in = 1:length(vtail_left_def)
    Rudder(in) = vtail_left_def(in) - vtail_right_def(in);
    Elevator_Left(in) = vtail_left_def(in)*2 - Rudder(in);
    Elevator_Right(in) = vtail_right_def(in)*2 + Rudder(in);
end
Elevator = -1*deg2rad(Elevator_Right');
Rudder = deg2rad(Rudder');

servotime = sliced_log.RCOU.TimeS-startTime;


throttle = throttle_def;
Aileron = Aileron_Data;
% Collecting Data
Control_Input = [0,Aileron(1),Elevator(1),Rudder(1),throttle(1); servotime, Aileron, Elevator, Rudder, throttle];


subplot(2,3,1)
plot(servotime,Adjust_aileron_right_def)
title("Right Wing")


subplot(2,3,5)
plot(servotime,deg2rad(vtail_left_def))
title("Left Tail")

Throttle = sliced_log.RCOU.C3;

subplot(2,3,3)
plot(servotime,throttle_def)
title("Throttle")

subplot(2,3,4)
plot(servotime,deg2rad(vtail_right_def))
title("Right Tail")


subplot(2,3,2)
plot(servotime,Adjust_aileron_left_def)
title("Left Wing")



%LATITUDE, LONGITUDE, AND ALTITUDE FROM GPS%

figure

lat = sliced_log.GPS.Lat;
long = sliced_log.GPS.Lng;
alt = sliced_log.GPS.Alt;
GPSTime = sliced_log.GPS.TimeS-startTime;

subplot(2,3,1)
plot(GPSTime,lat)
title("GPS Latitude")

subplot(2,3,2)
plot(GPSTime,long)
title("GPS Longitude")

subplot(2,3,3)
plot(GPSTime,alt)
title("GPS Altitude")

% intialize the constants needed
% Define initialized constants
%initial state @ trim point for straight and level flight @20m/s
x0 = [uvel(1); %inital speed
    vvel(1);
    wvel(1);
    deg2rad(p(1));
    deg2rad(q(1));
    deg2rad(r(1));
    deg2rad(rollangle(1));
    deg2rad(pitchangle(1)); 
    deg2rad(yawangle(1))];

%initial control surface deflections
%input IC trim point for straight and level flight @20m/s
%since we linearized at the original trim point, all control inputs are
%centered at zero
uo = [Aileron(1);
    Elevator(1);
    Rudder(1);
    throttle(1)];

%initial longitude and latitude @honolulu airport
lat0 = convert_coordinates(21, 18, 56.1708);
lon0 = convert_coordinates(157, 51, 29.1348);

%initial plane altitude (m)
h0 = alt(1);

% Define Actuator Saturation Limits
u1min = -25*pi/180;
u1max = 25*pi/180;

u2min = -25*pi/180;
u2max = 10*pi/180;

u3min = -30*pi/180;
u3max = 30*pi/180;

u4min = 0;
u4max = 1;
