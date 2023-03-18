function [vref,x] = adaptive_dymanic_UAV(vcp, vc, v, x,chi, k3, k4, L, ts)
%                                         vcp, vc, v, chies, K1, K2, L, ts                                   
%  Summary of this function goes here
%  Detailed explanation goes here
mu_l = v(1);
mu_m = v(2);
mu_n = v(3);
omega = v(4);
a = L(1);
b = L(2);
%% Gain Matrices
K3 = k3*eye(size(v,1));
K4 = k4*eye(size(v,1));
%% Control error veclocity
ve = vc-v;
control = vcp + K3*tanh(inv(K3)*K4*ve);
sigma_1=control(1);
sigma_2=control(2);
sigma_3=control(3);
sigma_4=control(4);
%% REFRENCE VELOCITIES
% vc_1 = vc(1);
% vc_2 = vc(2);
% vc_3 = vc(3);
% vc_4 = vc(4);

vc_1 = v(1);
vc_2 = v(2);
vc_3 = v(3);
vc_4 = v(4);
% %

Y = [sigma_1, sigma_4,     0,       0,     0,       0,       0,                   0,       0,   vc_1, vc_2*omega, a*omega*vc_4,               0,     0,               0,     0,             0,             0,       0;
         0,       0, sigma_2, sigma_4,     0,       0,       0,                   0,       0,      0,           0,               0,  vc_1*omega,  vc_2,    b*omega*vc_4,     0,             0,             0,       0;
         0,       0,     0,       0, sigma_3,       0,       0,                   0,       0,      0,           0,               0,           0,     0,               0,  vc_3,             0,             0,       0;
         0,       0,     0,       0,     0, b*sigma_1, a*sigma_2, sigma_4*(a^2 + b^2), sigma_4,    0,           0,               0,           0,     0,               0,     0,  a*vc_1*omega,  b*vc_2*omega, vc_4];
 
%% AAPTATIVE CONTROLLER
% [x] = adaptacion(x, Y, ve, ts);
% 
% vref = Y*x;

% 
K = 0.1*eye(19);
xp = inv(K)*Y'*ve;
x = x + xp*ts;
vref = Y*x;

% vref = Y*chi';



end
