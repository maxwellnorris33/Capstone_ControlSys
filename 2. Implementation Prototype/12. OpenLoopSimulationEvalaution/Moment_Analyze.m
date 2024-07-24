%script to parse the .bin ardupilot file into something that can be used by
%matlab
close all
warning off
clear
clc


% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
%  Section 1 - Reading the Data from the logs, into the code
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
march16 = Ardupilog('M16_FL1_SLF.bin');


% Sliced Data for the straight sections to examine the Lift Coefficent
a_speed  = {};

a_speed{1} = march16.ARSP.Airspeed(3127:3252,:);    % SLF 1
a_speed{2} = march16.ARSP.Airspeed(3472:3652,:);    % SLF 2
a_speed{3} = march16.ARSP.Airspeed(3932:4004,:);    % SLF 3
a_speed{4} = march16.ARSP.Airspeed(4282:4341,:);    % SLF 4


% Sliced data for the right banking regimes 
lower_slice = 450000;
upper_slice = 650000;

slice_type ='LineNo';
sliced_log = march16.getSlice([lower_slice, upper_slice], slice_type);
Roll_x = sliced_log.ATT.Roll;
Gyro_x = sliced_log.IMU.GyrX;
Gyro_y = sliced_log.IMU.GyrY;
Gyro_z = sliced_log.IMU.GyrZ;
Gyro_time = sliced_log.IMU.TimeS;

vtail_right_pwm = sliced_log.RCOU.C4;
vtail_left_pwm = sliced_log.RCOU.C2;
aileron_right_pwm = sliced_log.RCOU.C1;
aileron_left_pwm = sliced_log.RCOU.C5;

% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% End of Section 1
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 


% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
%  Section 2 - Map the control surface deflections to degrees
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
vtail_right_def = interp1([2120, 800],[24.5, -24], (vtail_right_pwm)); %1400 is trim
vtail_left_def = interp1([2200, 800],[-26, 23.5], (vtail_left_pwm)); %1510 is trim
aileron_right_def = interp1([1004, 2200],[16, -10], (aileron_right_pwm));%1600 is trim
aileron_left_def = interp1([907, 2000],[-11.5, 15.5], (aileron_left_pwm)); %1400 is trim
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% End of Section 2
% ---o---o---o---o---o---o---o---o---o---o---o---o---o---


%  --- This chunk of code plots the defelctions and rolls ---
figure()
subplot(3,1,1)
plot(aileron_left_def)
hold on
plot(aileron_right_def)
hold on
plot(vtail_left_def)
hold on 
plot(vtail_right_def)
title("Control Surface Deflection")
ylabel('Degree')
xlabel('Line Number')
xlim([250 1000])
legend("Left Ail.","Right Ail.","LeftTail","RigthTail")
subplot(3,1,2)
plot(Roll_x)
title("Bank Angle")
ylabel('Degrees')
xlabel('Line Number')
xlim([250 1000])
subplot(3,1,3)
plot(Gyro_x)
title("Angular Velocity about x axis")
xlabel('Line Number')
ylabel('rad/s')
xlim([2000 8000])



figure()
plot(Gyro_time,Gyro_x)
ttle = title('x-Gyroscope Data');
ttle.FontSize = 10;
xla = xlabel("Time in Seconds");
xla.FontSize =10;
yla =ylabel("(rad/s)");
yla.FontSize = 10;


% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% Section 3 - Data Extraction
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% Find the peaks of the data set, this will coorelate the impact of a
% deflection on the gyro (rad/s/s). This is tedious work

% Get the peaks of the data
[GPeakVal_x, GPeakIndex_x] = findpeaks(Gyro_x,'MinPeakHeight',0.4,'MinPeakProminence',0.4);

% Smooth the data, but mainin the peaks. This will allow is to estalish a
% referecne point from which we can differentiate to get angular
% acceleration as the gyroscope data is a angular velocity

xrange = mslowess(Gyro_time,Gyro_x);
trange = Gyro_time;

% Find where the data crosses a threshold in the upwards direction
zerocrossing = find(diff(sign(xrange >= 0.25)) ~= 0);
upwardcross = zerocrossing(xrange(zerocrossing) < 0.25);

% Tune the data, eliminating unwanted points. This section MUST BE DONE BY
% HAND to ensure you have the same number of data points as you do turns.
upwardcross(end) = [];


% Map the indices of the peak and reference data to the gyroscope arrays.
GLow_valx = xrange(upwardcross);
GLow_idxx = trange(upwardcross);

GHigh_valx = Gyro_x(GPeakIndex_x);
GHigh_idxx = trange(GPeakIndex_x);

%Create an array to store the accelertation. Using discrete integration by
%finding the change in angular velocity over a given time. 

alpha_x = [];
for i=1:length(GLow_idxx)
    dv = GHigh_valx(i) - GLow_valx(i);
    dt = GHigh_idxx(i) - GLow_idxx(i);
    accel = dv./dt;
    alpha_x(i) = accel;
end
alpha_x = alpha_x';

% This marks the end of the x_direction gyroscope data. The steps are
% repeated for the Y, and the Z directions below. The goal is to find the
% peak angular velocity of the turn and a bese reference. Then we find the
% slope thus the acceleration.


GPeakVal_y = [];
GPeakIndex_y = [];

trange = Gyro_time(2500:8500);
yrange = Gyro_y(2500:8500);
yrange_s = mslowess(Gyro_time(2500:8500),yrange);
[GY1v,GY1i] = findpeaks(Gyro_y(2500:8500),'MinPeakProminence',0.1,'MinPeakDistance',400,'MinPeakHeight',0.16);
GPeakVal_y = [GPeakVal_y;GY1v];
GPeakIndex_y = [GPeakIndex_y;GY1i];


zerocrossing = find(diff(sign(yrange_s >= 0)) ~= 0);
upwardcross = zerocrossing(yrange_s(zerocrossing) < 0);
upwardcross(1) = [];
upwardcross(5) = 3800;
upwardcross(6:10) = [];
upwardcross(7:end) = [];
GLow_valy = yrange_s(upwardcross);
GLow_idxy = trange(upwardcross);

GHigh_valy = Gyro_y(GPeakIndex_y);
GHigh_idxy = trange(GPeakIndex_y);


alpha_y = [];
for i=1:length(GLow_idxy)
    dv = GHigh_valy(i) - GLow_valy(i);
    dt = GHigh_idxy(i) - GLow_idxy(i);
    accel = dv./dt;
    alpha_y(i) = accel;
end
alpha_y = alpha_y';

alpha_y(7:16) = 0;


% Please note that the Y-direction gyroscope data does not impact the CM_x,
% the data is extremely noisy so the data collection is very meticulous
% here. Also in a bank, theres not much...if any y angular velocity for a
% rolling turn anyway. 

zrange = mslowess(Gyro_time,Gyro_z);
[GPeakVal_z, GPeakIndex_z] = findpeaks(Gyro_z,'MinPeakHeight',0.2,'MinPeakDistance',200,'MinPeakProminence',0.2);
[zsmoothz, zsmootthi] = findpeaks(zrange,'MinPeakHeight',0.1,'MinPeakDistance',200,'MinPeakProminence',0.2);


zerocrossing = find(diff(sign(zrange >= 0.25)) ~= 0);
upwardcross = zerocrossing(zrange(zerocrossing) < 0.25);
upwardcross(12) = [];
upwardcross(14) = [];
upwardcross(14) = [];
upwardcross(17) = [];

GLow_valz = zrange(upwardcross);
GLow_idxz = Gyro_time(upwardcross);

GHigh_valz = Gyro_z(GPeakIndex_z);
GHigh_idxz = Gyro_time(GPeakIndex_z);

alpha_z = [];
for i=1:length(GLow_idxz)
    dv = GHigh_valz(i) - GLow_valz(i);
    dt = GHigh_idxz(i) - GLow_idxz(i);
    accel = dv./dt;
    alpha_z(i) = accel;
end
alpha_z = alpha_z';

%
% Now we transition to finding the control surface deflections that cause
% the aircraft rotation. Unfortunately we cannot extrude the data from the
% timestep at which the gyroscope data spikes as these will be a delay in
% actuation and response caused by damping. So, a similar method is used
% below. 
%

[AL_pv, AL_pi] = findpeaks(-aileron_left_def,"MinPeakHeight",0,'MinPeakDistance',50,'MinPeakProminence',2);
[AR_pv, AR_pi] = findpeaks(aileron_right_def,'MinPeakProminence',2,'MinPeakDistance',50);

% Peak finder was too dificult to use to read this data. So it was analyzed
% by hand using MATLAB data tips

RT_pv = [2.23409
        2.82197
        3.48333
        3.18939
        2.38106
        4.21818
        1.05833
        1.94015
        1.7197
        2.01364
        3.26288
        5.46742
        4.07121
        5.68788
        5.98182
        5.76136
        ];

LT_pv = [2.80571
         2.16929
         3.01786
         3.37143
         2.80571
         -0.73
         1.53286
         1.88643
         2.94714
         2.94714
         2.87643
         1.03786
         -0.800714
         -0.659286
         -0.447143
         -0.447143
        ];
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% End of Section 3
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 




%  --- This chunk of code plots the defelctions and rolls ---
figure()
subplot(3,1,1)
plot(aileron_left_def)
hold on
plot(aileron_right_def)
hold on
plot(vtail_left_def)
hold on 
plot(vtail_right_def)
title("Signal")
legend("LeftW","RightW","LeftTail","RigthTail")
subplot(3,1,2)
plot(Roll_x)
title("Roll deg")
subplot(3,1,3)
plot(Gyro_x)
title("Alpha about x axis")
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% THIS PLOTTING CHUNK PLOTS THE DEFELCTIONS IN PWM INSTEAD OF DEGREES
% figure('Position',[1000 200 800 800])
% subplot(5,1,1)
% plot(aileron_left_pwm)
% title('Left Wing')
% subplot(5,1,2)
% plot(aileron_right_pwm)
% title('Right Wing')
% subplot(5,1,3)
% plot(Gyro_time,Gyro_x)
% title('Gyro X')
% subplot(5,1,4)
% plot(Gyro_time,Gyro_y)
% title("Gyro Y")
% subplot(5,1,5)
% plot(Gyro_time,Gyro_z)
% title('Gyro Z')
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 



% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% Section 4 - Interprete the Data
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 



% 4.1 - Comapring Lift Coefficient
vel = [];

for i = 1:length(a_speed)
    vel(i) = mean((a_speed{i}));
end


sttd = [];
for i = 1:length(a_speed)
    sttd(i) = std((a_speed{i}));
end

emperical = 0.5281;


mass = 5.73;                % Wet Mass is kg
g0 = 9.81;                  % Gravitaional Constant
R_air = 287;                % Ideal Gas of AIR
T0 = 0.74 + 273.15;         % Static Temperature Kelvin        
S_plan = 2.176 * 0.222;     % Planform Area m^2
P_0 = 676.4 .* 0133.322     % mmHg to kPa
rho = P_0./(R_air.*T0);


disp("Average vel")
disp(vel)
disp("Lift Coefficient for SLF at Ardupilot target 20m/s")
Cl = (2.*g0.*mass.*R_air.*T0)./(S_plan.*P_0.*(vel.^2));
disp(Cl)
q = 0.5.*rho.*(mean(vel).^2);


FlightNumber = [1, 2, 3, 4]';
Velocity_Average_mps = vel';
Standard_Deviation = sttd';
Cl = Cl';
error = abs((Cl - emperical)./emperical).*100;
Error_Compared_to_CFD = error;

Cl_table = table(FlightNumber,Velocity_Average_mps,Standard_Deviation,Cl,Error_Compared_to_CFD);
disp(Cl_table)






% 4.2 Comparing Moment Coefficients

alpha =zeros(3,16);


alpha(1,:) = (alpha_x);
alpha(2,:) = alpha_y;
alpha(3,:) = (alpha_z);

CM = zeros(16,3);
I_tensor  = 5.73.* [0.0802 0        0.108 
                    0      0.175    0
                    0.108  0        0.108];

for i = 1:length(alpha)
    CM(i,:) = (I_tensor * alpha(:,i))./(243.42.*0.5122*0.222);
end

banks = [1:16]';
Wing_lef = -AL_pv;
Wing_rit = AR_pv;
RUDD_lef = -LT_pv;
RUDD_rit = RT_pv;

CM_x = CM(:,1);
CM_y = CM(:,2);
Cm_z = CM(:,3);

ave_def = [];
for i=1:length(banks)
    ave_def(i,1) = (mean([abs(Wing_rit(i)),abs(Wing_lef(i))]));
    ave_def(i,2) = (mean([abs(RUDD_rit(i)),abs(RUDD_lef(i))]));
end

expectedCM = getCM(ave_def(:,1),ave_def(:,2))';

aveAileron = ave_def(:,1);
aveRudder = ave_def(:,2);
CMx_analytical = expectedCM(:,1);
CMy_analytical = expectedCM(:,2);
CMz_analytical = expectedCM(:,3);


CM_table = table(banks,aveAileron,aveRudder,CM_x,CMx_analytical, CM_y, CMy_analytical, Cm_z,CMz_analytical);
disp("Considering Yaw, Pitch, and Roll rates: ")
disp(CM_table)


figure()
plot(banks,CM_x,'o-b')
hold on
plot(banks,CMx_analytical,'x-k')

title('Analyzed CM_x for right banking flight')
xlabel('Bank Number')
ylabel("CM_x")

lgd = legend("From the Flight Data","Interpolated from CFD");
lgd.FontSize=11;
xlim([0 20])
ylim([0 0.2])

CM_xerr = abs(CM_x - CMx_analytical)./(0.01.*CMx_analytical);
mean(CM_xerr)












































% - o - o - o - o - o - o - Khanhs Code - o - o - o - o - o - o - 
% Its just uncommented so I dont change anything


%slice_type = 'LineNo';
%log = Ardupilog();
%sliced_log = log.getSlice([lower_slice, upper_slice], slice_type);
%airspeed = sliced_log.ARSP.Airspeed;

% thrust_pwm = sliced_log.RCOU.C3;
% vtail_right_pwm = sliced_log.RCOU.C4;
% vtail_left_pwm = sliced_log.RCOU.C2;
% aileron_right_pwm = sliced_log.RCOU.C1;
% aileron_left_pwm = sliced_log.RCOU.C5;

%convert avg vtail pwms to deflection angles

%note, actual deflections are estimated at the moment, will need actual
%measurements to map PWM to. Right now the mapping indicates that trim is
%not 0 deflection.

% vtail_right_def = interp1([800, 2120],[24.5, -24], mean(vtail_right_pwm)); %1400 is trim
% vtail_left_def = interp1([2200, 800],[-26, 23.5], mean(vtail_left_pwm)); %1510 is trim
% 
% %convert avg aileron pwms to deflection angles
% 
% %no data for mapping yet, need max min pwm and corresponding deflections
% aileron_right_def = interp1([2200, 1004],[16, -10], mean(aileron_right_pwm));%1600 is trim
% aileron_left_def = interp1([2000, 907],[-11.5, 15.5], mean(aileron_left_pwm)); %1400 is trim
