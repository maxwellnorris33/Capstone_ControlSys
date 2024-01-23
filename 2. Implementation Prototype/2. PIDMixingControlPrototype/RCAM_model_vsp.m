function [XDOT] = RCAM_model_vsp(X,U)

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

u1 = U(1); %d_A (aileron)
u2 = U(2); %d_T (stabilizer)
u3 = U(3); %d_R (rudder)
u4 = U(4); %d_th1 (throttle 1)
%u5 = U(5); %d_th2 (throttle 2)

%--------------------------CONSTANTS---------------------------------------
%Nominal Vehicle constants
m = 1.9979; %aircraft total mass (kg) - no electronics
%NOTE: we will define Ib and invIb later

cbar = 0.22283;             %mean aerodynamic chord (m)
lt = 0.5;                  %distance between AC and tail (m) - assumption
S = 0.51220761;             %wing planform area (m^2)
%St = 0.2;                   %tail planform area (m^2) - assumption cause chase did not calculate

Xcg = 0.443;                %x position of CoG in Fm (m)
Ycg = 0;                    %y position of CoG in Fm (m)
Zcg = 0.10*cbar;            %z position of CoG in Fm (m)

Xac = 0.44836;            %x position of AC in Fm (m)
Yac = 0;                    %y position of AC in Fm (m)
Zac = 0;                    %z position of AC in Fm (m)

%Engine Constants
Xapt1 = 0;                  %x position of engine 1 force in Fm (m)
Yapt1 = 0;              %y position of engine 1 force in Fm (m)
Zapt1 = -1.9;               %z position of engine 1 force in Fm (m)

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
% %calculate the CL_wb (wing and body)
% if alpha<=alpha_switch
%     CL_wb = n*(alpha-alpha_L0);
% else
%     CL_wb = a3*alpha^3 + a2*alpha^2 +a1*alpha +a0;
% end
% 
% %calculate CL_t (tail)
% epsilon = depsda*(alpha-alpha_L0);
% alpha_t = alpha - epsilon +u2 + 1.3*x5*lt/Va;
% CL_t = 3.1*(St/S)*alpha_t;

%total lift force
CL = 0.539+5.24*alpha;

%Total Drag Force 
CD = 0.0367 + 0.302*alpha;

%Calculating Sideforce
CY = -0.17*beta-0.0423*u3;

%----------------------4. DIMENSIONAL AERODYNICAL FORCES-------------------
%calculate the actual dimensional forces. These are in Fs

FA_s = [-CD*Q*S; CY*Q*S; -CL*Q*S];

%rotate these forces to Fb
C_bs = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)];
FA_b = C_bs*FA_s;

%---------------------5. AERODYNAMIC MOMENT COEFFICIENT ABOUT AC-----------

%calculated cl, cm, cn values
cl = 0.518*beta + 0.292*u1;
cm = 0.0623 - 0.126*alpha + 2.651*u2;
cn = -0.0403*beta + 0.105* u3;
%CM about aerodynamic centre in body frame created
CMac_b = [cl;cm;cn];

%--------------------6. AERODYNAMIC MOMENT ABOUT AC------------------------
%normalize to aerodynamic moment
MAac_b = CMac_b*Q*S*cbar;


%-------------------7. AERODYNAMIC MOMENT ABOUT CG-------------------------
%transfer moment to cg
rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac; Yac; Zac];
MAcg_b = MAac_b + cross(FA_b, rcg_b - rac_b);

%-------------------8. ENGINE FORCE AND MOMENT-----------------------------
% Effect of engine. Calculate thrust force of engine
if u4<22
    F1 = u4*2.1-0.18;
else
    F1 = (0.16-0.871*u4+6.42*u4^2)*g;
end


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
Ib = m*[40.07 0 -2.0923; 0 64 0; -2.0923 0 99.92];

%inverse of inertia matrix
invIb = (1/m)*[0.0249836 0 0.000523151; 0 0.015625 0; 0.000523151 0 0.010019];

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





