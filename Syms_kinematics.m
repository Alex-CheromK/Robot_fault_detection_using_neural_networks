% This script uses MATLAB's symbolic processor to solve Lagrange's
% equations and generate the kinetic equations of motion for the robot
% manipulator.

%% Setup workspace
clear;
clc;

%% Define symbolic variables
% Link angles
syms gamma(t) theta1(t) theta2(t)
% Link angular velocities
syms gamma_dot(t) theta1_dot(t) theta2_dot(t)
% Link angular accelerations
syms gamma_dot_dot(t) theta1_dot_dot(t) theta2_dot_dot(t)
% Robot and other parameters
syms L1 L2 m g real

%% Define kinematic variables
% Define vectors that are aligned with each link at zero angle positions.
r1_init = [L1 0 0]';
r2_init = [L2 0 0]';
% Note: These vectors will be rotated used transformation matrices to
% determine each link's "current" position.

% Define rotation matrices associated with each motor rotation...
% Robot yaw
R_gamma = [
    cos(gamma) -sin(gamma) 0;
    sin(gamma) cos(gamma) 0;
    0 0 1];
% Link 1 pitch
R_theta1 = [
    cos(theta1) 0 sin(theta1);
    0 1 0;
    -sin(theta1) 0 cos(theta1)];
% Link 2 pitch
R_theta2 = [
    cos(theta2) 0 sin(theta2);
    0 1 0;
    -sin(theta2) 0 cos(theta2)];

%% Kinematics
% Define the position vector of the robot's package as a function of the
% link angles
r_m = R_gamma*R_theta1*(r1_init + R_theta2*r2_init);
% Differentiate the above expression to compute the package velocity
v_m = subs(diff(r_m,t),[diff(gamma,t) diff(theta1,t) diff(theta2,t)],[gamma_dot theta1_dot theta2_dot]);
% Note: The "subs()" function is used to replace derivative terms like
% diff(gamma,t) with easier-to-read terms like gamma_dot.

%% Solve Lagrange's equations
% Define the kinetic and potential energies of the system
T = (1/2)*m*(v_m.'*v_m);
V = m*g*([0 0 1]*r_m);

% Solve Lagrange's equation for each robot DOF
L_gamma = simplify(subs(...
    diff(diff(T,gamma_dot),t) - diff(T,gamma) + diff(V,gamma),...
    [diff(gamma,t) diff(theta1,t) diff(theta2,t) diff(gamma_dot,t) diff(theta1_dot,t) diff(theta2_dot,t)],...
    [gamma_dot theta1_dot theta2_dot gamma_dot_dot theta1_dot_dot theta2_dot_dot]));
L_theta1 = simplify(subs(...
    diff(diff(T,theta1_dot),t) - diff(T,theta1) + diff(V,theta1),...
    [diff(gamma,t) diff(theta1,t) diff(theta2,t) diff(gamma_dot,t) diff(theta1_dot,t) diff(theta2_dot,t)],...
    [gamma_dot theta1_dot theta2_dot gamma_dot_dot theta1_dot_dot theta2_dot_dot]));
L_theta2 = simplify(subs(...
    diff(diff(T,theta2_dot),t) - diff(T,theta2) + diff(V,theta2),...
    [diff(gamma,t) diff(theta1,t) diff(theta2,t) diff(gamma_dot,t) diff(theta1_dot,t) diff(theta2_dot,t)],...
    [gamma_dot theta1_dot theta2_dot gamma_dot_dot theta1_dot_dot theta2_dot_dot]));

