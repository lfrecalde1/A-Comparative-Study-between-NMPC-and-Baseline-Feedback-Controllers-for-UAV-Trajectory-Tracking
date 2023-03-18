function [Test,x] = adaptive_dymanic_UAV2(x, vcp, vc, v, qd, q, A, L, ts)
%                                         vcp, vc, v, chies, K1, K2, L, ts                                   
%  Summary of this function goes here
%  Detailed explanation goes here

a = L(1);
b = L(2);
%% Gain Matrices
A = A*eye(size(v,1));
%% Control error veclocity
ve = vc -v;
qe = qd - q;
sigma = ve + A*qe;

vr = sigma + v;
vrp = vcp + A*ve;

vr_1 = vr(1);
vr_2 = vr(2);
vr_3 = vr(3);
vr_4 = vr(4);
omega = vr(4);

vrp_1=vrp(1);
vrp_2=vrp(2);
vrp_3=vrp(3);
vrp_4=vrp(4);
%% REFRENCE VELOCITIES


Yu = [vrp_1, vrp_4,     0,       0,     0,       0,       0,                   0,       0,   vr_1, vr_2*omega, a*omega*vr_4,               0,     0,               0,     0,             0,             0,       0;
         0,       0, vrp_2, vrp_4,     0,       0,       0,                   0,       0,      0,           0,               0,  vr_1*omega,  vr_2,    b*omega*vr_4,     0,             0,             0,       0;
         0,       0,     0,       0, vrp_3,       0,       0,                   0,       0,      0,           0,               0,           0,     0,               0,  vr_3,             0,             0,       0;
         0,       0,     0,       0,     0, b*vrp_1, a*vrp_2, vrp_4*(a^2 + b^2), vrp_4,    0,           0,               0,           0,     0,               0,     0,  a*vr_1*omega,  b*vr_2*omega, vr_4];
 
%% AAPTATIVE CONTROLLER

K = 1*eye(19);

xp = K*Yu'*sigma;
x = x + xp*ts;


Test = Yu*x;

end
