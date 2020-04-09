% Justin Francis
% ECE 5780, Embedded Sys Design
% Dr. Gaillardon, U of U
% Lab 7, DC Motor Controls

clc; clear; close all;

%% Prob 7.1, 2
close all;

figure();
hold on;

t = linspace(0,8);
u = ones(size(t));

for i = 1:10
    K = i; %torq const, [Nm/A]
    J = i; %rot inert, [kg/m2]
    R = i; %resistance, [ohm]
    sys = tf([1], [J*R/K, K]);
    [Y, T, X] = lsim(sys, u, t);
    plot(t,Y, 'DisplayName', strcat('K = J = R = ',num2str(i)));
    legend('-DynamicLegend');
    legend('show');
    drawnow;
    
end

title('Justin Francis, Frictionless Step-Response');
xlabel('Time, t[s]');
ylabel('Velocity (Amplitude), v[m/s]');
legend();
grid();



%% 7.2
clc; close; clear all;

K_i = 0;
K_p = 1;
V_s = 1;
B = 1;
J = 1;
K = 1;
R = 1;

sim('Lab7_DCMotorControl.slx');

figure();
hold on;
plot(rotVel, 'LineWidth', 2);
xlabel('Time, t[s]'); 
ylabel('Velocity, \theta[rad/s]');

%% 7.2 
openfig('MultipleFrictionSimulation.fig');
title('Dynamically Changed Friction at t = 5[s]');
xlabel('Time, t[s]');
ylabel('Motor Velocity, \omega[rad/s]');
%%
openfig('P_Controller.fig');
title('Dynamically Changed Friction at t = 5[s] with P Controller');
xlabel('Time, t[s]');
ylabel('Motor Velocity, \omega[rad/s]');
%%
openfig('PI_Controller.fig');
title('Dynamically Changed Friction at t = 5[s] with PI Controller');
xlabel('Time, t[s]');
ylabel('Motor Velocity, \omega[rad/s]');
