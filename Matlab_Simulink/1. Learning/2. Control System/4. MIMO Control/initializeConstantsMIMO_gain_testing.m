%this script generates the reduced system statespace and the K gains for
%the MIMO control system. These variables will be generated into the
%workspace which will then be automatically saved for use in other scripts

clear 
close all
clc

%extracting statespace A, B, C and D matrix
linearizedSS = load('rcam_linearized_ss@85ms_straight_and_level.mat'); %varname is linsys1

A = linearizedSS.linsys1.A;
B = linearizedSS.linsys1.B;
C = linearizedSS.linsys1.C;
D = linearizedSS.linsys1.D;

%synthesizing matlab statespace system

sys = ss(A, B, C, D);

%truncating system
rsys = modred(sys, [2;4;6;7;9], "Truncate"); 
% only states x1 (uvel), x2 (zvel), x5 (pitch rate), x8 (pitch ang). x10 (altitude) remaining 
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

%plot poles
%pzplot(rsys)

%will need to trial and error with the below five poles to see which will
%yield a K matrix which will behave the way we want it to with the reduced
%system

p1 = -0.11;
p2 = -0.01;
p3 = -0.2;
p4 = -0.2;
p5 = -0.15;

K = place(A, B, [p1, p2, p3, p4, p5])
%close loop system with new K controller
cloop_sys = ss(A-B*K, B, C, D, 0);
pzplot(cloop_sys)

save('k_gains', "K")

%if the above plot is stable, save the the above K gains into file to be
%used in the simulation


%getting statespace from the full model linearization
linear_sys = load("rcam_linearized_ss@85ms_straight_and_level.mat").linsys1;
A = linear_sys.A;
B = linear_sys.B;
C = linear_sys.C;
D = linear_sys.D;

% Define initialized constants
%initial state @ trim point for straight and level flight @85m/s
x0 = [85; %inital speed
    0;
    1.268;
    0;
    0;
    0;
    0;
    0.0149; 
    0;
    500];

%initial control surface deflections
%input IC trim point for straight and level flight @85m/s
%since we linearized at the original trim point, all control inputs are
%centered at zero
uo = [0;
    -0.17797;
    0;
    0.16418];

TF = 20*60; %how long the sim runs for

%k gain
k_gain = load("k_gains.mat");
k = k_gain.K;

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



