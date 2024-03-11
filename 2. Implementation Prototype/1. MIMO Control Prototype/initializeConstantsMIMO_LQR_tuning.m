%keilan changes in case i lose

%this script generates the reduced system statespace and the K gains for
%the MIMO control system. These variables will be generated into the
%workspace which will then be automatically saved for use in other scripts

clear 
close all
clc

%extracting statespace A, B, C and D matrix
linearizedSS = load('rcam_linearized_ss@20ms_straight_and_level.mat'); %varname is linsys1

A = linearizedSS.linsys1.A;
B = linearizedSS.linsys1.B;
C = linearizedSS.linsys1.C;
D = linearizedSS.linsys1.D;

%synthesizing matlab statespace system
sys = ss(A, B, C, D);

%truncating system
rsys = modred(sys, [2;3;4;6;7;8;9], "Truncate"); 
% x1 (uvel), x5 pitch rate, x10 (altitude) remaining as these will be measurable
% states

% will use the reduced system to design K gains matrix
A = rsys.A;
B = rsys.B;
C = rsys.C;
D = rsys.D;

%checking observability and controllability
Co = ctrb(A, B);
Ob = obsv(A, C);

if rank(Co) == rank(Ob) & (length(A) - rank(Co)) == 0 & (length(A) - rank(Ob)) == 0;
    disp('rsys is observable and controllable');
else
    disp('rsys is NOT observable or controllable');
end

%LQR tuning
Sys = ss(rsys.A,rsys.B,rsys.C,rsys.D);
Q_Sys = [0.001 0 0; 
    0 0 0; 
    0 0 10];

R_Sys = [1 0 0 0; 
    0 16 0 0; 
    0 0 1 0;
    0 0 0 970];

[P,~,~] = care(Sys.A,Sys.B,Q_Sys,R_Sys);
K_LQR = -inv(R_Sys)*Sys.B'*P;

%saving LQR K matrix
save('k_gains_LQR', "K_LQR")

%getting statespace from the full model linearization
linear_sys = load("rcam_linearized_ss@20ms_straight_and_level.mat").linsys1;
A = linear_sys.A;
B = linear_sys.B;
C = linear_sys.C;
D = linear_sys.D;

% Define initialized constants
%initial state @ trim point for straight and level flight @85m/s
x0 = [20; %inital speed
    0;
    0.11625;
    0;
    0;
    0;
    0;
    0.0058125; 
    0];

%initial control surface deflections
%input IC trim point for straight and level flight @85m/s
%since we linearized at the original trim point, all control inputs are
%centered at zero
uo = [0;
    0.050707;
    0;
    0.35514];

TF = 10*60; %how long the sim runs for

%k gain
k_gain_LQR = load("k_gains_LQR.mat")
k = k_gain_LQR.K_LQR

%initial longitude and latitude @honolulu airport
lat0 = convert_coordinates(21, 18, 56.1708);
lon0 = convert_coordinates(157, 51, 29.1348);

%initial plane altitude (m)
h0 = 50;

% Define Actuator Saturation Limits
u1min = -25*pi/180;
u1max = 25*pi/180;

u2min = -25*pi/180;
u2max = 10*pi/180;

u3min = -30*pi/180;
u3max = 30*pi/180;

u4min = 0;
u4max = 1;



