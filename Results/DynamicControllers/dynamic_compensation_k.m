function [vref] = dynamic_compensation_k(vc_k, vc, v, A, B, k3, ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
mu_l = v(1);
mu_m = v(2);
mu_n = v(3);
omega = v(4);

%% Gain Matrices
K3 = k3*eye(size(v,1));

%% Control error veclocity
ve = vc-v;
control = (1/ts)*(vc_k-K3*ve-v);


vref = pinv(B)*(control)-pinv(B)*A*vc;
%% AAPTATIVE CONTROLLER


end