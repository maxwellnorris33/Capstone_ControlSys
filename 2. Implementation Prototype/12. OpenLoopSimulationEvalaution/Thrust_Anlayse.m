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
Gyro_time = sliced_log.IMU.TimeS;



lower_slice = 420000;
upper_slice = 472000;
slice_type ='LineNo';
sliced_log2 = march16.getSlice([lower_slice, upper_slice], slice_type);


imu_x = sliced_log2.IMU.AccX;
imu_x = imu_x -0.2;                % Zero the IMU Data
imu_t = sliced_log2.IMU.TimeS;
thrust_pwm = sliced_log2.RCOU.C3;
si_time = sliced_log2.RCOU.TimeS;
vel_x = sliced_log2.ARSP.Airspeed;             
vel_t = sliced_log2.ARSP.TimeS;
elev = sliced_log2.BARO.Alt;
elev_t = sliced_log2.BARO.TimeS;
roll_x = sliced_log2.ATT.Roll;
roll_t = sliced_log2.ATT.TimeS;

master_time = imu_t;

% 
% figure()
% plot()

% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% End of Section 1
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 


% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
%  Section 2 - Map the control surface deflections to degrees
%              And the Throttle values
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 

% Map the Throttle
thrust_percent = interp1([1100 1900],[0 1],(thrust_pwm));


% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
%  Section 3 - 
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
mass = 5.73;                % Wet Mass is kg
g0 = 9.81;                  % Gravitaional Constant
R_air = 287;                % Ideal Gas of AIR
T0 = 4.4 + 273.15;          % Static Temperature Kelvin        
S_plan = 2.176 * 0.222;  % Planform Area m^2
P_0 = 676.4 .* 0133.322;    % mmHg to kPa
P_0 = 88933;
rho = P_0./(R_air.*T0);


low = 4070;
high = 4075;


imu_xdata = getDataAtTime(low, high, imu_t, imu_x,0.01);
thrust_percent_data = getDataAtTime(low, high, si_time,thrust_percent,0.05);
vel_data = getDataAtTime(low,high,vel_t,vel_x,0.2);

low = 4058;
high = 4058.5;
% 2

imu_xgodata = getDataAtTime(low, high,imu_t,imu_x,0.01);
thrust_percent_godata = getDataAtTime(low, high, si_time,thrust_percent,0.05);
vel_godata = getDataAtTime(low,high,vel_t,vel_x,0.21);

%Thurst data from THRUSTTEST Straight section
F_data1 = 1.57 - 8.55.*(thrust_percent_data(:,2)) + 63.*(thrust_percent_data(:,2).^2);

% Straight Regime (Regime 1)
Fs1 = getStaticF(thrust_percent_data(:,2));
Cd1 = (2.*Fs1)./(rho.*((vel_data(:,2)).^2).*S_plan);

% Try using Jareds Drag

% Cd_Jared = 0.07;
% Cd1 = linspace(Cd_Jared,Cd_Jared,length(Cd1));
% Cd1 = Cd1';

Freg1 = 0.5.*rho.*vel_data(:,2).^2 .*S_plan .* Cd1;

% Acceleration Regime (Regime 2)
Fs2 = getStaticF(thrust_percent_godata(:,2));
Cd2 = ((2)./(rho.*((vel_godata(:,2)).^2).*S_plan)) .* (Fs2 - (5.73.*(imu_xgodata(:,2))));
Freg2 = 5.73.*(imu_xgodata(:,2))  + 0.5.*S_plan.*rho.*((vel_godata(:,2)).^2) .*(Cd2);

for i=1:length(Cd2)
    if Cd2(i) <= 0
        disp('Nope Try AGAIN, we got a Negative')
    else
    end
end

multiplier = 0.894;
Flow = multiplier.*(getStaticF(thrust_percent_godata(:,2)));

Cd2low = ((2)./(rho.*((vel_godata(:,2)).^2).*S_plan)) .* (Flow - (5.73.*(imu_xgodata(:,2))));
disp("Lowest DRAG coeff in Regime 2")
disp(min(Cd2low))


figure()
StaticCurve = @(t) (1.57 -8.55.*t + 63.*(t.^2));
MinThrustCurve = @(t) 0.894.*(1.57 -8.55.*t + 63.*(t.^2));
fplot(StaticCurve,[0 1])
hold on
fplot(MinThrustCurve,[0 1])

title("Thrust Curves Used for Analysis")
legend("Thrust Curve from Static Thrust Test (F_{t,s})","Shifted Curve (89.4% of F_{t,s})")
xlabel("Throttle Commanded")
ylabel("Thrust Force [N]")



cd_steadymax = (2.*StaticCurve(mean(thrust_percent_data(:,2))))./(rho.*((vel_data(:,2)).^2).*S_plan);
cd_steadymin = (2.*MinThrustCurve(mean(thrust_percent_data(:,2))))./(rho.*((vel_data(:,2)).^2).*S_plan);
mean(cd_steadymin)
mean(cd_steadymax)







figure()
plot(vel_godata(:,2),Cd2)
hold on
plot(vel_data(:,2),Cd1,'^b','MarkerSize',10)
xlabel("Vel")
ylabel("Cd")
xlim([0 25])
ylim([-0.5 4])

SL_VSPsim = 34.7;
SL_CFDsim = 51;
SL_data   = mean(thrust_percent_data(:,2)).*100;
figure()
thrust_xaxis = categorical({'Flight Data','CFD Simulation','OpenVSP Simulation'});
bar(thrust_xaxis,[SL_data 100; SL_CFDsim NaN;SL_VSPsim NaN],0.7)
hold on 
title("Throttle Percentage for Different Flight Regimes ")
legend('Steady Level Fligth','Accelerating Regime')
ylabel('Percentage Throttle')
ylim([0 110])
yticks([0:10:100])



disp('Static Thurst in Regime 1 is: ')
disp(mean(Fs1))
disp('Equivilent Thurst in Regime 1 is: ')
disp(mean(Freg1))
disp('Static Thurst in Regime 2 is: ')
disp(min(Fs2))
disp('Minimum thrust in Regime 2 is: ')
disp(min(Freg2))
disp('Drag Coefficient in Regime 1 is: ')
disp(mean(Cd1))
disp('Drag Coefficient in Regime 2 is: ')
disp(mean(Cd2))






% Fth = F_data1;
% Cd_data = (2.*Fth)./(1.225.*(mean(vel_data(:,2)).^2));
% guess = 0.6;
% Cd_guess = linspace(guess,guess,length(Cd_data));
% Cd_guess = Cd_guess';
% condition = 1;
% counter = 0;
% while condition == 1
% 
%     % Find Drag Coefficient
% 
%     % Find thrust from EOM
%     F_calc1  = 5.73.*(imu_xdata(:,2))  + 0.5.*S_plan.*rho.*((mean(vel_data(:,2))).^2) .*mean(Cd_guess);
%     Fth = linspace(Fth(1),Fth(end),length(F_calc1));
%     Fth = Fth';
% 
%     % Find Drag Coefficient
%     Cd_calc = (2.*F_calc1)./(1.225.*(mean(vel_data(:,2)).^2).*S_plan);
% 
% 
%     Cd_err = Cd_calc - Cd_guess;
% 
%     F_err = F_calc1 - Fth;
% 
% 
%     if abs(mean(F_err)) < 0.01
% 
%         condition = 0;
%     end
% 
%     Fth = 5.73.*mean(imu_xdata(:,2))  + 0.5.*S_plan.*rho.*((mean(vel_data(:,2))).^2) .*(Cd_calc);
%     counter = counter + 1;
% 
%     Cd_guess = (Cd_guess + Cd_calc )./2;
% 
% end
% disp("Loop took ")
% disp(counter)
% disp('Ierations')
% 


% Using Drag, find thrust at Accelerating section
% F_calc2  = 5.73.*(imu_xgodata(:,2))  + 0.5.*S_plan.*rho.*((mean(vel_data(:,2))).^2) .*mean(Cd_data);
% 
% %Find thrust in Accelecration section using THRUSTTEST
% F_data2 = 1.57 - 8.55.*(thrust_percent_godata(:,2)) + 63.*(thrust_percent_godata(:,2).^2);
% F_data2 = linspace(F_data2(1),F_data2(end),length(F_calc2));
% F_data2 = F_data2';
% 
% 

% figure()
% plot([1:length(Fth)],Fth)
% hold off
% 
% 
% F_err = F_calc2 - F_data2;
% 
% 
% disp('Drag Coefficient')
% disp(mean(Cd_data))








% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
% Section 4 - Interprete the Data
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 



% 4.1 - Comapring Lift Coefficient
vel = [];
for i = 1:length(a_speed)
    mean_vel = mean(a_speed{i});
    vel(i) = mean_vel;
end


mass = 5.73;                % Wet Mass is kg
g0 = 9.81;                  % Gravitaional Constant
R_air = 287;                % Ideal Gas of AIR
T0 = 4.4 + 273.15;          % Static Temperature Kelvin        
S_plan = 2.176 * 0.222;  % Planform Area m^2
P_0 = 676.4 .* 0133.322;    % mmHg to kPa
% P_0 = 88933;
rho = P_0./(R_air.*T0);


disp("Average vel")
disp(vel)
disp("Lift Coefficient for SLF at Ardupilot target 20m/s")
Cl = (2.*g0.*mass.*R_air.*T0)./(S_plan.*P_0.*(vel.^2));
disp(Cl)
q = 0.5.*rho.*(mean(vel).^2);



% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 
%  Section 5 - Plot Stuff
% ---o---o---o---o---o---o---o---o---o---o---o---o---o--- 



% figure()
% subplot(5,1,1)
% plot(si_time,thrust_percent)
% title('Thrust %')
% subplot(5,1,2)
% plot(imu_t,(imu_x))
% title("Accel DATA")
% subplot(5,1,3)
% plot(vel_t,vel_x)
% title('Velocity')
% subplot(5,1,4)
% plot(elev_t,elev)
% title("Altitude")
% subplot(5,1,5)
% plot(roll_t,roll_x)
% title('Roll')



figure()
subplot(2,1,1)
plot(si_time,thrust_percent.*100)
title('Percentage of Throttle Commanded')
xlabel('Time (s)')
ylabel("Percentage")
xlim([4057 4060])
ylim([0 120])
subplot(2,1,2)
plot(imu_t,(imu_x))
title("Acceleration in the x -direction")
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
xlim([4057 4060])



% 
% 
% vtail_right_pwm = sliced_log.RCOU.C4;
% vtail_left_pwm = sliced_log.RCOU.C2;
% aileron_right_pwm = sliced_log.RCOU.C1;
% aileron_left_pwm = sliced_log.RCOU.C5;





function thrust = getStaticF(throttlepercentage)
    t = throttlepercentage;
    mod = 1;
    thrust = mod.*(1.57 -8.55.*t + 63.*(t.^2));

end

function  returnvar = getDataAtTime(t_interest1,t_interest2,t_int,var_int,tol)

    if (t_interest1 > t_interest2)
        disp("Time Span entry is Invalid")
        return
    end

    timeStart_index = find(abs(t_int-t_interest1)<tol);
    timeEnd_index = find(abs(t_int-t_interest2)<tol);
    a = length(timeStart_index) -1;
    b = length(timeEnd_index) -1;
    timeStart_index = timeStart_index(1);
    timeEnd_index = timeEnd_index(end);
    time = t_int(timeStart_index:timeEnd_index);
    data = var_int(timeStart_index:timeEnd_index);

    % Make all return values the same length
    time = linspace(time(1),time(end),2000);
    data = linspace(data(1),data(end),2000);
    time = time';
    data = data';

    returnvar = [time,data];

end




























