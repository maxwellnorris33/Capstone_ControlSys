%run this script to initialize variables for RCAM_MIMO_implementation
clear 
clc
close all

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
    1000];

%initial control surface deflections
%input IC trim point for straight and level flight @85m/s
%since we linearized at the original trim point, all control inputs are
%centered at zero
uo = [0;
    0;
    0;
    0];

TF = 100; %how long the sim runs for

%k gain
k_gain = load("k_gains.mat");
k = k_gain.K;




