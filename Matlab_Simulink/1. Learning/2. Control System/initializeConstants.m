%initialize constants for RCAM Simulation
clear 
clc
close all

% Define initialized constants
%initial state @ trim point for straight and level flight @100m/s
x0 = [75; %inital speed
    0;
    -3.6407;
    0;
    0;
    0;
    0;
    -0.036415; 
    0];

% x0 = [84.9905; 0; 1.2713; 0; 0; 0; 0; 0.0150; 0];
% uo = [0; -0.1780; 0; 0.0821; 0.0821];

%initial control surface deflections
%input IC trim point for straight and level flight @100m/s
uo = [0;
    -0.3    3109;
    0;
    0.097743];
    %0.097743];

%control surface disturbance preset (simulated step input   
udist = [10;
    -10;
    10;
    5]* pi/180;
    %5] * pi/180;

TF = 10*60; %how long the sim runs for

%initial longitude and latitude @honolulu airport
lat0 = convert_coordinates(21, 18, 56.1708);
lon0 = convert_coordinates(157, 51, 29.1348);

%initial plane altitude (m)
h0 = 1000;

% Define Actuator Saturation Limits
u1min = -25*pi/180;
u1max = 25*pi/180;

u2min = -25*pi/180;
u2max = 10*pi/180;

u3min = -30*pi/180;
u3max = 30*pi/180;

u4min = 0;
u4max = 10*pi/180;

%u5min = 0;
%u5max = 10*pi/180;



% run model
% sim_results = sim('RCAM_simulation.slx');


