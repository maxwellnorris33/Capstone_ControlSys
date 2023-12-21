function [XDOT] = rsys_model(X, U)

reduced_sys = load('reduced_ss@85ms_straight_and_level.mat').rsys; %input the reduced model here
XDOT = reduced_sys.A*X + reduced_sys.B*U;

end

