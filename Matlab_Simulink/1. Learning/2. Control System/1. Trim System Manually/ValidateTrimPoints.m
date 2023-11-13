clear
clc 
close all

%load trim point
temp = load('trim_values_straight_level.mat');
XStar = temp.XStar;
UStar = temp.UStar;

TF = 250;
out = sim("RCAM_simulation.slx")

t = out.simX.Time;
X = out.simX.Data;

figure;

for k = 1:9
    subplot(5,2,k)
    plot(t, X(:,k), 'LineWidth', 2)
    ylabel(['x_', num2str(k)])
    grid on
end
