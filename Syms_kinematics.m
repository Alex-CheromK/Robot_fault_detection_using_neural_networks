clear;
clc;

syms gamma(t) theta1(t) theta2(t)
syms gamma_dot(t) theta1_dot(t) theta2_dot(t)
syms gamma_dot_dot(t) theta1_dot_dot(t) theta2_dot_dot(t)
syms L1 L2 m g real

r1_init = [L1 0 0]';
r2_init = [L2 0 0]';

R_gamma = [
    cos(gamma) -sin(gamma) 0;
    sin(gamma) cos(gamma) 0;
    0 0 1];
R_theta1 = [
    cos(theta1) 0 sin(theta1);
    0 1 0;
    -sin(theta1) 0 cos(theta1)];
R_theta2 = [
    cos(theta2) 0 sin(theta2);
    0 1 0;
    -sin(theta2) 0 cos(theta2)];

r_m = R_gamma*R_theta1*(r1_init + R_theta2*r2_init);
v_m = subs(diff(r_m,t),[diff(gamma,t) diff(theta1,t) diff(theta2,t)],[gamma_dot theta1_dot theta2_dot]);

T = (1/2)*m*(v_m.'*v_m);
V = m*g*([0 0 1]*r_m);

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

