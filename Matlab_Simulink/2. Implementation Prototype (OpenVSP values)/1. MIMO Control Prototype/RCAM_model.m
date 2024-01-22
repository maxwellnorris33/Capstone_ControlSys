function [XDOT] = RCAM_model(X,U)

%--------------------------STATE AND CONTROL VECTORS-----------------------
%Extract state vectors
x1 = X(1); %u
x2 = X(2); %v
x3 = X(3); %w
x4 = X(4); %p
x5 = X(5); %q
x6 = X(6); %r
x7 = X(7); %phi
x8 = X(8); %theta
x9 = X(9); %psi
% x10 = X(10); %PN
% x11 = X(11); %PE
% x12 = X(12); %PD

u1 = U(1); %d_A (aileron) +is right roll
u2 = U(2); %d_T (stabilizer) +is down deflection
u3 = U(3); %d_R (rudder) +is right deflection
u4 = U(4); %d_th1 (throttle 1)
%u5 = U(5); %d_th2 (throttle 2)

%--------------------------CONSTANTS---------------------------------------
%Nominal Vehicle constants
m = 7; %aircraft total mass (kg) %NEED THIS
%NOTE: we will define Ib and invIb later

cbar = 0.22283;                 %mean aerodynamic chord (m)
S = 0.51220761;                    %wing planform area (m^2)
b = 2.176;

Xcg = 0.0934;            %x position of CoG in Fm (m) %THIS IS REFERENCE TO THE NOSE, NEED TO SUBTRACT BY DISTANCE TO THE LEADING EDGE FOR FM
Ycg = 0;                    %y position of CoG in Fm (m)
Zcg = 0;            %z position of CoG in Fm (m) %NEED THIS

Xac = 0.09856;            %x position of AC in Fm (m)
Yac = 0;                    %y position of AC in Fm (m)
Zac = 0;                    %z position of AC in Fm (m)

%Engine Constants %NEED THIS
Xapt1 = 0.840;                  %x position of engine 1 force in Fm (m)
Yapt1 = 0;              %y position of engine 1 force in Fm (m)
Zapt1 = 0;               %z position of engine 1 force in Fm (m)

%Other constants
rho = 1.225;                %air density (kg/m^3)
g = 9.81;                   %gravitational acceleration (m/s^2)

%--------------------------2. INTERMEDIATE VARIABLES---------------------
%calculate airspeed
Va = sqrt(x1^2 +x2^2 +x3^2);

%calculate alpha and beta
alpha = atan2(x3, x1);
beta = asin(x2/Va);

%calculate dynamic pressure
Q = 0.5*rho*Va^2;

%define vectors wbe_b and V_b
wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];


%----------------------3. AERODYNAMIC FORCE COEFFICIENTS-------------------
%coeffs from openVSP is in windframe, need to rotate to body frame

%total lift force
CL = 0.539+5.240*alpha;

%Total Drag Force (neglecting tail)
CD = 0.0367+0.302*alpha;

%Calculating Sideforce
CY = -0.176*beta - 0.0423*u3;

%----------------------4. DIMENSIONAL AERODYNICAL FORCES-------------------
%calculate the actual dimensional forces. These are in Fs

FA_s = [-CD*Q*S; CY*Q*S; -CL*Q*S];

%rotate these forces to Fb
C_bs = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)];
FA_b = C_bs*FA_s;

%--------------------6. AERODYNAMIC MOMENT ABOUT CG------------------------
%normalize to aerodynamic moment about cog
MAcg_b = [(0.518*beta+0.292*u1)*b; (0.0623-0.126*alpha+2.651*u2)*cbar; (-0.0403*beta+0.105*u3)*b]*Q*S;

%m/qsc=c

% rcg_b = [Xcg;Ycg;Zcg];
% rac_b = [Xac; Yac; Zac];
% MAcg_b = MAac_b + cross(FA_b, rcg_b - rac_b);

%-------------------8. ENGINE FORCE AND MOMENT-----------------------------
% Effect of engine. Calculate thrust force of engine

if u4<0.22
    F1 = u4*2.1-0.18;
else
    F1 = 6.457*u4^2-0.9309*u4+0.1751;
end
F1 = u4*g; %convert thrust test curve from kg to N

%assuming engine thrust is aligned with Fb, we have
FE1_b = [F1;0;0];

FE_b = FE1_b; 

%now engine moment due to offset of engine thrust from CG
mew1 = [Xcg - Xapt1; Yapt1 - Ycg; Zcg - Zapt1];

MEcg1_b = cross(mew1, FE1_b);

MEcg_b = MEcg1_b;
%-------------------9. GRAVITY EFFECTS------------------------------------
g_b = [-g*sin(x8); g*cos(x8)*sin(x7); g*cos(x8)*cos(x7)];

Fg_b = m*g_b;

%-------------------10. STATE DERIVATIVES---------------------------------
%inertia matrix
Ib = m*[100 0 -10; 0 600 0; -10 0 1000]; %NEED THIS

%inverse of inertia matrix
invIb = (1/m)*[0.0249836 0 0.000523151; 0 0.015625 0; 0.000523151 0 0.010019];
invIb = Ib^-1;

%form f_b and calculate udot, vdot, wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b, V_b);

%form Mcg_b and calculate pdot, qdot and rdot
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b, Ib*wbe_b));

%calculate phidot, thetadot and psidot
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8); 0 cos(x7) -sin(x7); 0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9dot = H_phi*wbe_b;

%place in first order form

XDOT = [x1to3dot; x4to6dot; x7to9dot];
end





