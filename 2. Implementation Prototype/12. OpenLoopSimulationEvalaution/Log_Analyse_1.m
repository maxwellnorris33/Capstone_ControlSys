%script to parse the .bin ardupilot file into something that can be used by
%matlab
close all
warning off
clear
clc


march16 = Ardupilog('M16_FL1_SLF.bin');
                               
a_speed  = {};

a_speed{1} = march16.ARSP.Airspeed(3127:3252,:);    % SLF 1
a_speed{2} = march16.ARSP.Airspeed(3472:3652,:);    % SLF 2
a_speed{3} = march16.ARSP.Airspeed(3932:4004,:);    % SLF 3
a_speed{4} = march16.ARSP.Airspeed(4282:4341,:);    % SLF 4


lower_slice = 450000;
upper_slice = 650000;
slice_type ='LineNo';
sliced_log = march16.getSlice([lower_slice, upper_slice], slice_type);
Roll_x = sliced_log.ATT.Roll;
Gyro_x = sliced_log.IMU.GyrX;
Gyro_y = sliced_log.IMU.GyrY;
Gyro_z = sliced_log.IMU.GyrZ;

vtail_right_pwm = sliced_log.RCOU.C4;
vtail_left_pwm = sliced_log.RCOU.C2;
aileron_right_pwm = sliced_log.RCOU.C1;
aileron_left_pwm = sliced_log.RCOU.C5;


vtail_right_def = interp1([1150, 1810],[24.5, -24], (vtail_right_pwm)); %1400 is trim
vtail_left_def = interp1([1100, 1800],[-26, 23.5], (vtail_left_pwm)); %1510 is trim
aileron_right_def = interp1([2000, 907],[16, -10], (aileron_right_pwm));%1600 is trim
aileron_left_def = interp1([2200, 1004],[-11.5, 15.5], (aileron_left_pwm)); %1400 is trim

% Examine the X Gyro Data and Aileron Data
[GPeakVal_x, GPeakIndex_x] = findpeaks(Gyro_x,'MinPeakHeight',0.5,'MinPeakProminence',0.4);


GPeakVal_y = [];
GPeakIndex_y = [];

[GY1v,GY1i] = findpeaks(Gyro_y(2500:8500),'MinPeakProminence',0.1,'MinPeakDistance',400,'MinPeakHeight',0.16);
GPeakVal_y = [GPeakVal_y;GY1v];
GPeakIndex_y = [GPeakIndex_y;GY1i];
GPeakVal_y(5) = mean([GPeakVal_y(4), GPeakVal_y(6)]);
[GY2v, GY2i] = findpeaks(Gyro_y(1.2e4: 1.6e4),'MinPeakProminence',0.05,'MinPeakDistance',400,'MinPeakHeight',0.09);
GPeakVal_y = [GPeakVal_y;GY2v];
GPeakIndex_y = [GPeakIndex_y;GY2i];
[GY3v, GY3i] = findpeaks(smoothdata(Gyro_y(1.8e4:2.6e4)),'MinPeakProminence',0.05,'MinPeakDistance',400,'MinPeakHeight',0.09);
GPeakVal_y = [GPeakVal_y;GY3v];
GPeakIndex_y = [GPeakIndex_y;GY3i];

[GPeakVal_z, GPeadIndex_z] = findpeaks(Gyro_z,'MinPeakHeight',0.2,'MinPeakDistance',200,'MinPeakProminence',0.2);




[AL_pv, AL_pi] = findpeaks(aileron_left_def,"MinPeakHeight",7.5,'MinPeakDistance',50,'MinPeakProminence',2);
[AR_pv, AR_pi] = findpeaks(-aileron_right_def,'MinPeakProminence',2,'MinPeakDistance',50);


RT_pv = [1.173409
        2.32197
        2.98333
        2.68939
        1.88106
        3.71818
        0.631818
        1.44015
        1.2197
        1.51364
        2.76288
        4.96742
        3.57121
        5.18788
        5.48182
        5.26136];

LT_pv = [0.305714
        -0.330714
        0.517857
        0.871429
        0.305714
        -3.23
        -1.03786
        -0.613571
        0.376429
        0.305714
        0.376429
        -1.46214
        -3.30071
        -3.15929
        -2.94714
        -2.94714];



%  --- This chunk of code plots the defelctions and rolls ---
% figure('Position',[1000 200 800 800])
% subplot(3,1,1)
% plot(aileron_left_def)
% hold on
% plot(aileron_right_def)
% hold on
% plot(vtail_left_def)
% hold on 
% plot(vtail_right_def)
% title("Signal")
% legend("LeftW","RightW","LeftTail","RigthTail")
% subplot(3,1,2)
% plot(Roll_x)
% title("Roll deg")
% subplot(3,1,3)
% plot(Gyro_y)
% title("Alpha about Y axis")
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
figure()
subplot(5,1,1)
plot(aileron_left_pwm)
xlim([300 1000])
title('Left Wing')
subplot(5,1,2)
plot(aileron_right_pwm)
title('Right Wing')
xlim([300 1000])
subplot(5,1,3)
plot(vtail_left_pwm)
title('Left Tail')
xlim([300 1000])
subplot(5,1,4)
plot(vtail_right_pwm)
title("Right Tail")
xlim([300 1000])
subplot(5,1,5)
plot(Roll_x)
title('Roll')
xlim([300 1000])
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 






vel = [];
for i = 1:length(a_speed)
    mean_vel = mean(a_speed{i});
    vel(i) = mean_vel;
end


mass = 5.73;                % Wet Mass is kg
g0 = 9.81;                  % Gravitaional Constant
R_air = 287;                % Ideal Gas of AIR
T0 = 4.4 + 273.15;          % Static Temperature Kelvin        
S_plan = 2.176 * 0.222;     % Planform Area m^2
P_0 = 676.4 .* 0133.322;    % mmHg to kPa
P_0 = 88933;

disp("Average vel")
disp(vel)
disp("Lift Coefficient for SLF at Ardupilot target 20m/s")
Cl = (2.*g0.*mass.*R_air.*T0)./(S_plan.*P_0.*(vel.^2));
disp(Cl)


alpha =zeros(3,16);
alpha(1,:) = GPeakVal_x;
alpha(2,:) = GPeakVal_y;
alpha(3,:) = GPeakVal_z;

CM = zeros(16,3);
I_tensor  = 5.73.* [0.0802 0        0.108 
                    0      0.175    0
                    0.108  0        0.108];

for i = 1:length(alpha)
    CM(i,:) = (I_tensor * alpha(:,i))./S_plan;
end

banks = [1:16]';
LeftAil = AL_pv;
RightAil = AR_pv;
RUDD_lef = LT_pv;
RUDD_rit = -RT_pv;

CM_x = CM(:,1);
CM_y = CM(:,2);
Cm_z = CM(:,3);

CM_table = table(banks,LeftAil,RightAil, RUDD_lef, RUDD_rit,CM_x, CM_y, Cm_z);
disp("Considering Yaw, Pitch, and Roll rates: ")
disp(CM_table)

figure()
plot(banks,LeftAil)
hold on
plot(banks,RightAil)
hold on
%plot(CM_x)



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
