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
rsys = modred(sys, [3;4;6;7;9], "Truncate"); 
% only states x1 (uvel), x2 (zvel), x5 (pitch rate), x8 (pitch angle), x10 (altitude) remaining 
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
p1 = -0.25 + 0.5i;
p2 = -0.25 - 0.5i;
p3 = -0.6;
p4 = -0.5;
p5 = -0.7;

K = place(A, B, [p1, p2, p3, p4, p5]);

%close loop system with new K controller
%cloop_sys = ss(A-B*K, B, C, D, 0);
%pzplot(cloop_sys)

save('k_gains', "K")


%if the above plot is stable, save the the above K gains into file to be
%used in the simulation

