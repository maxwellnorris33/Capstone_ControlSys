function [rudder_angle, elevator_angle] = VtoTtailv2(Langle, Rangle)

%----------------------- assumptions and definitions -----------------

% L and R angles are the angled of each control surfaces servo with (-)
% assumed to be folded in/up and (+) assumed to be folded out/down

% outputs are resulting elevator and rudder angles in a traditional T
% shaped tail configuration

% Conversion credit to "Small unmanned aircraft Theory and Practice by:
% Randal W. Beard and Timothy W. McLain

% Point of refference is behind the UAV looking into the page on the tail
% -> nose axis

% all angles are in radians

%--------------------------- Calculations ----------------------------

%defining variables
VTail_angles = [Rangle; Langle];
Conversion_mat = [1 1; -1 1];

%performing conversion
RudElv_angles = Conversion_mat*VTail_angles;


%setting output variables
elevator_angle = RudElv_angles(1,1);
rudder_angle = RudElv_angles(2,1);

end