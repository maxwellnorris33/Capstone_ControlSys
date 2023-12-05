function [Rangle, Langle] = TtoVtail(rudder_angle, elevator_angle)

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
Tail_angles = [elevator_angle; rudder_angle];
Conversion_mat = [1 1; -1 1]^-1;

%performing conversion
RudElv_angles = Conversion_mat*Tail_angles;


%setting output variables
Rangle = RudElv_angles(1,1);
Langle = RudElv_angles(2,1);

end