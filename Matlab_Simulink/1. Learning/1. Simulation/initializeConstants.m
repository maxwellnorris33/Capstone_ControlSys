%initialize constants for RCAM Simulation
clear 
clc
close all

% Define initialized constants
%initial state @ trim point for straight and level flight @235m/s
% x0 = [232.4717; %inital speed
%     -0.0033194;
%     -34.3786;
%     0;
%     1.825e-17;
%     0;
%     0.00010293;
%     -0.14682; %approx 5.73 deg pitch up
%     0];

x0 = [84.9905; 0; 1.2713; 0; 0; 0; 0; 0.0150; 0];

uo = [0; -0.1780; 0; 0.0821; 0.0821];

%initial control surface deflections
%input IC trim point for straight and level flight @235m/s
% uo = [0;
%     -0.027228;
%     0;
%     0.49711;
%     0.49711];

%control surface disturbance preset (simulated step input   
udist = [10;
    -10;
    10;
    5;
    5] * pi/180;

TF = 60*60; %how long the sim runs for

%initial longitude and latitude @honolulu airport
lat0 = convert_coordinates(21, 18, 56.1708);
lon0 = convert_coordinates(157, 51, 29.1348);

%initial plane altitude (m)
h0 = 0;

% Define Actuator Saturation Limits
u1min = -25*pi/180;
u1max = 25*pi/180;

u2min = -25*pi/180;
u2max = 10*pi/180;

u3min = -30*pi/180;
u3max = 30*pi/180;

u4min = 0;
u4max = 10*pi/180;

u5min = 0;
u5max = 10*pi/180;



% run model
% sim_results = sim('RCAM_simulation.slx');


