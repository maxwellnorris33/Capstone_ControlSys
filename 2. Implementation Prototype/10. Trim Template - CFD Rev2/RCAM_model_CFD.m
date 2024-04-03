%tutorial from christopher lum
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

%--------------------------CONSTANTS---------------------------------------
%Nominal Vehicle constants
m = 5.73; %aircraft total mass (kg) %NEED THIS

cbar = 0.22283;                 %mean aerodynamic chord (m)
S = 0.51220761;                    %wing planform area (m^2)
b = 2.176;

Xcg = 0.0641;            %x position of CoG in Fm (m) 
Ycg = 0;                    %y position of CoG in Fm (m)
Zcg = -0.04;            %z position of CoG in Fm (m)

Xac = 0.0822;            %x position of AC in Fm (m)
Yac = 0;                    %y position of AC in Fm (m)
Zac = 0;                    %z position of AC in Fm (m)

%Engine Constants %NEED THIS
Xapt1 = 0.75531;                 %est rn %x position of engine 1 force in Fm (m)
Yapt1 = 0;              %y position of engine 1 force in Fm (m)
Zapt1 = -0.02;               %z position of engine 1 force in Fm (m)

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
if u2 >= 0
    CL = (0.568 + 4.83*alpha - 7.74*alpha^2) ... %CL wrt angle of attack
        + (7.784*x5) ... %CL wrt pitch rate
        + (-0.446265515867565 * u2); %CL wrt elevator
else
    CL = (0.568 + 4.83*alpha - 7.74*alpha^2) ... %CL wrt angle of attack
        + (7.784*x5) ... %CL wrt pitch rate
        + (-0.428640876296062 * u2); %CL wrt elevator
end

%Total Drag Force 
if u2 >=0
    CD = (0.124 + 0.812*alpha + 4.53*alpha^2) ... %CD wrt angle of attack
        + (0.202*x5) ... %CD wrt pitch rate
        + (0.0102540050707052 * u2); %CD wrt elevator
else
    CD = (0.124 + 0.812*alpha + 4.53*alpha^2) ... %CD wrt angle of attack
        + (0.202*x5) ... %CD wrt pitch rate
        + (-0.022900465888788 * u2); %CD wrt elevator
end
%Total Sideforce
% OpenVSP: CY = -0.176*(beta) - 0.0423*u3;
CY = 0.381855937977578*beta ... %CY wrt sideslip
    - 0.0198*x4 ... %CY wrt roll rate
    - 0.184*x6 ... %CY wrt yaw rate
    - 0.165392960537476*u3 ... %CY wrt rudder
    + 0.0273768615093121*u1; %CY wrt aileron

%----------------------4. DIMENSIONAL AERODYNICAL FORCES-------------------
%calculate the actual dimensional forces. These are in Fw

FA_w = [-CD*Q*S; CY*Q*S; -CL*Q*S];

%rotate these forces to Fb
C_bw = [cos(beta)*cos(alpha) -sin(beta)*cos(alpha) -sin(alpha); sin(beta) cos(beta) 0; cos(beta)*sin(alpha) -sin(beta)*sin(alpha) cos(alpha)];
FA_b = C_bw*FA_w;

%--------------------6. AERODYNAMIC MOMENT ABOUT CG------------------------
%normalize to aerodynamic moment about cog

%these moments are in the wind frame, need to rotate to body frame
% OpenVSP: MAcg_w = [(0.0498849*beta + 0.5630736*x4 + 0.2814754*u1); %roll
%     (0.0567244-0.1194170*(alpha) - 8.9756*x5 + -1.1049921*u2); %pitch
%     (-0.0395123*beta + 0.0595315*x6 -0.0207352*u3)]*Q*S*cbar; %yaw

if u2 >=0
    MAcg_w = [(0.5630736*x4 + 0.155*x6 - 0.0446985307848685*beta + 0.427015721674515*u1 + 0.0422366661337774*u3); %roll
        (-8.9756*x5 - 0.00599 - 0.197*alpha + 0.199352667127091*u2); %pitch
        (0.0595315*x6 - 0.0281*x4 + 0.154163878358508*beta - 0.0264143821157647*u1 - 0.0921527135445746*u3)]*Q*S*cbar; %yaw
else
    MAcg_w = [(0.5630736*x4 + 0.155*x6 - 0.0446985307848685*beta + 0.427015721674515*u1 + 0.0422366661337774*u3); %roll
        (-8.9756*x5 - 0.00599 - 0.197*alpha + 0.228014491962055*u2); %pitch
        (0.0595315*x6 - 0.0281*x4 + 0.154163878358508*beta - 0.0264143821157647*u1 - 0.09215271354457462*u3)]*Q*S*cbar; %yaw
end

%rotated moments to body frame
MAcg_b = C_bw*MAcg_w ;

%-------------------8. ENGINE FORCE AND MOMENT-----------------------------
% Effect of engine. Calculate thrust force of engine
if u4 <= 0
    F1 = 0;
elseif u4<0.22 & u4>0
    F1 = 1.2*u4;
else
    F1 = 6.457*u4^2-0.9309*u4+0.1751;
end

F1 = F1*g; %convert thrust test curve from kg to N

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
Ib = m*[0.0802 0 0.108; 
        0 0.175 0; 
        0.108 0 0.1084]; %NEED THIS

%inverse of inertia matrix
invIb = (1/m)*[-36.4944 0 36.3597; 
    0 5.7143 0; 
     36.3597 0 -27.0005];

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





